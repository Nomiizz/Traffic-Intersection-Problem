/*
 * traffic.c
 *
 * 09-2015: KMS: created
 *
 */


/*
 *  CS350 Students Note!
 *
 *  You may not modify the code in this file in any way!
 *
 */


/*
 * 
 * Includes
 *
 */

#include <types.h>
#include <lib.h>
#include <test.h>
#include <clock.h>
#include <thread.h>
#include <synch.h>
#include <synchprobs.h>
#include <lamebus/ltimer.h>
#include <kern/errno.h>

/*
 * Problem parameters
 *
 * Values for these parameters are set by the main driver
 *  function based on the problem parameters
 *  that are passed in from the kernel menu command or
 *  kernel command line.
 *
 * These are only ever modified by one thread, when the simulation
 * is being initialized, so they do not need to be volatile.
 */

#define MAX_THREADS 10

static int NumIterations = 100;  // number of vehicle arrivals per thread
static int NumThreads = 10;      // number of concurrent simulation threads
static int InterArrivalTime = 1;      // time between vehicle arrivals
static int ServiceTime = 1;    // time in the intersection
static int DirectionBias = 0;  // 0 = unbiased, 1 = biased

/*
 * Once the main driver function has created the 
 * simulation threads, it uses this semaphore to block until all of the
 * simulation threads are finished.
 */
static struct semaphore *SimulationWait;

/*
 *
 * shared simulation state
 * 
 * note: this is state should be used only by the 
 *  functions in this file, hence the static declarations
 *
 */

/* note: this typedef is defined here because it is used only
   within this file, and is not exposed externally */

typedef struct Vehicles
{
  Direction origin;
  Direction destination;
} Vehicle;

/*  vehicles in the intersection */
/*  note: this is an array of volatile pointers */
static Vehicle * volatile vehicles[MAX_THREADS];
/* semaphore to protect vehicles array */
static struct semaphore *mutex;

/* performance statistics, one set per origin direction */
static volatile time_t total_wait_secs[4];
static volatile uint32_t total_wait_nsecs[4];
static volatile int wait_count[4];
static volatile time_t max_wait_secs[4];
static volatile uint32_t max_wait_nsecs[4];
/* mutex to provide mutual exclusion to performance stats */
static struct semaphore *perf_mutex;
/* number of milliseconds per timer tick (LT_GRANULARITY is in usec) */
#define TICK_MILLISEC (LT_GRANULARITY/1000)

/* simulation start and end time */
time_t start_sec, end_sec;
uint32_t start_nsec, end_nsec;

/* bias direction, for arrival biasing */
Direction heavy_direction;

/* functions defined and used internally */
static void initialize_state(void);
static void cleanup_state(void);
static void print_direction(Direction d);
static void print_perf_stats(void);
static void vehicle_simulation(void *ptr, unsigned long thread_num);
static bool right_turn(Vehicle *v);
static void check_constraints(int thread_num);


/* 
 * print_direction()
 * 
 * Purpose:
 *   print a direction
 *
 * Arguments:
 *   a direction
 *
 * Returns:
 *   nothing
 */
static void
print_direction(Direction d) {
  switch (d)
    {
    case north:
      kprintf("N");
      break;
    case east:
      kprintf("E");
      break;
    case south:
      kprintf("S");
      break;
    case west:
      kprintf("W");
      break;
    }
}    

/* 
 * choose_direction()
 * 
 * Purpose:
 *   randomly choose a direction, with or without bias
 *
 * Arguments:
 *   none
 *
 * Returns:
 *   a direction
 */
static Direction
choose_direction(void) {
  int x;
  if (DirectionBias) {
    /* this gives 70% arrivals from the heavy direction,
       and 10% from each of the other directions */
    x = random()%10;
    if (x >= 4) {
      /* 60% from the heavy direction */
      return heavy_direction;
    }
    else {
      /* otherwise choose any direction (including heavy) with equal prob */
      return x;
    }
  }
  else {
    return random()%4;
  }
}    


/* 
 * print_perf_stats()
 * 
 * Purpose:
 *   print the performance statistics
 * 
 * Arguments: 
 *   none
 * 
 * Returns:
 *   nothing
 */
static void
print_perf_stats(void) {
  int i;
  int max_allowed_ms;
  int wait_msecs,mean_wait_msecs,max_wait_msecs;
  int total_wait_msecs = 0;
  int total_count = 0;
  int sim_msec;
  time_t run_sec;
  uint32_t run_nsec;

  /* report maximum permitted wait time */
  /* arbitrary standard: we are willing to wait one service time
     for per other thread in the simulation */
  max_allowed_ms = (NumThreads-1)*ServiceTime*TICK_MILLISEC;
  kprintf("max permitted wait:\t%d.%03d seconds\n",
	  max_allowed_ms/1000,max_allowed_ms%1000);
  /* print average wait time for each direction of origin */
  for(i=0;i<4;i++) {
    print_direction((Direction)i);
    kprintf(":\t");
    if (wait_count[i] > 0) {
      wait_msecs = (total_wait_secs[i]*1000+total_wait_nsecs[i]/1000000);
      total_wait_msecs += wait_msecs;
      // some rounding error here, in millisecond range
      mean_wait_msecs = wait_msecs/wait_count[i];
      total_count += wait_count[i];
      max_wait_msecs = max_wait_secs[i]*1000+max_wait_nsecs[i]/1000000;
      kprintf("%d vehicles, average wait %d.%03d seconds, max wait %d.%03d seconds\n",
	      wait_count[i], mean_wait_msecs/1000,mean_wait_msecs%1000,
	      max_wait_msecs/1000,max_wait_msecs%1000);
    } else {
      kprintf("0 vehicles, average wait 0.000 seconds, max wait 0.000 seconds\n");
    }
  }
  /* then the average wait time for all vehicles */
  if (total_count > 0) {
    kprintf("all:\t%d vehicles, average %d.%03d seconds waiting\n",total_count,
	    (total_wait_msecs/total_count)/1000,
	    (total_wait_msecs/total_count)%1000);
  } else{
    kprintf("all:\t0 vehicles, average 0.000 seconds waiting\n");
  }
  /* finally, overall simulation run-time and throughput */
  getinterval(start_sec,start_nsec,end_sec,end_nsec,&run_sec,&run_nsec);
  sim_msec = run_sec*1000;
  sim_msec += run_nsec/1000000;
  kprintf("Simulation: %d.%03d seconds, %d vehicles\n",
	  sim_msec/1000,
	  sim_msec%1000,
	  total_count);
} 


/*
 * bool right_turn()
 * 
 * Purpose:
 *   predicate that checks whether a vehicle is making a right turn
 *
 * Arguments:
 *   a pointer to a Vehicle
 *
 * Returns:
 *   true if the vehicle is making a right turn, else false
 *
 * Note: written this way to avoid a dependency on the specific
 *  assignment of numeric values to Directions
 */
bool
right_turn(Vehicle *v) {
  KASSERT(v != NULL);
  if (((v->origin == west) && (v->destination == south)) ||
      ((v->origin == south) && (v->destination == east)) ||
      ((v->origin == east) && (v->destination == north)) ||
      ((v->origin == north) && (v->destination == west))) {
    return true;
  } else {
    return false;
  }
}


/*
 * check_constraints()
 * 
 * Purpose:
 *   checks whether the entry of a vehicle into the intersection violates
 *   any synchronization constraints.   Causes a kernel panic if so, otherwise
 *   returns silently
 *
 * Arguments:
 *   number of the simulation thread that moved the vehicle into the intersection 
 *
 * Returns:
 *   nothing
 */

void
check_constraints(int thread_num) {
  int i;
  KASSERT(thread_num < NumThreads);
  /* compare newly-added vehicle to each other vehicles in in the intersection */
  for(i=0;i<NumThreads;i++) {
    if ((i==thread_num) || (vehicles[i] == NULL)) continue;
    /* no conflict if both vehicles have the same origin */
    if (vehicles[i]->origin == vehicles[thread_num]->origin) continue;
    /* no conflict if vehicles go in opposite directions */
    if ((vehicles[i]->origin == vehicles[thread_num]->destination) &&
        (vehicles[i]->destination == vehicles[thread_num]->origin)) continue;
    /* no conflict if one makes a right turn and 
       the other has a different destination */
    if ((right_turn(vehicles[i]) || right_turn(vehicles[thread_num])) &&
	(vehicles[thread_num]->destination != vehicles[i]->destination)) continue;
    /* Houston, we have a problem! */
    /* print info about the two vehicles found to be in conflict,
       then cause a kernel panic */
    kprintf("Vehicle A: ");
    print_direction(vehicles[i]->origin);
    kprintf("->");
    print_direction(vehicles[i]->destination);
    kprintf("\n");
    kprintf("Vehicle B: ");
    print_direction(vehicles[thread_num]->origin);
    kprintf("->");
    print_direction(vehicles[thread_num]->destination);
    kprintf("\n");
    panic("intersection synchronization constraint violation!\n");
  }
}

/*
 * initialize_state()
 * 
 * Purpose:
 *   initializes simulation 
 *
 * Arguments:
 *   none
 *
 * Returns:
 *   nothing
 */

static void
initialize_state(void)
{
  int i;
  for(i=0;i<MAX_THREADS;i++) {    
    vehicles[i] = (Vehicle * volatile)NULL;
  }
  for(i=0;i<4;i++) {
    total_wait_secs[i] = total_wait_nsecs[i] = wait_count[i] = 0;
    max_wait_secs[i] = max_wait_nsecs[i] = 0;
  }
  mutex = sem_create("Vehicle Mutex",1);
  if (mutex == NULL) {
    panic("could not create vehicle mutex semaphore\n");
  }
  perf_mutex = sem_create("PerfMutex",1);
  if (perf_mutex == NULL) {
    panic("could not create perf_mutex semaphore\n");
  }
  SimulationWait = sem_create("SimulationWait",0);
  if (SimulationWait == NULL) {
    panic("could not create SimulationWait semaphore\n");
  }
  heavy_direction = random()%4;
  /* initialization for synchronization code */
  intersection_sync_init();

}


/*
 * cleanup_state()
 * 
 * Purpose:
 *   Releases simulation resources
 *
 * Arguments:
 *   None
 *
 * Returns:
 *   Nothing
 */

static void
cleanup_state(void)
{
  sem_destroy(mutex);
  sem_destroy(perf_mutex);
  sem_destroy(SimulationWait);
  intersection_sync_cleanup();
}

/*
 * in_intersection()
 *
 *  simulate time spent passing through the intersection
 *
 * Arguments: 
 *    none
 *
 * Return:
 *    nothing
 */

static void
in_intersection(void) {
   clocknap(ServiceTime);
}


/*
 * vehicle_simulation()
 *
 * Simulates the arrival of vehicles to the intersection
 *   from one direction.
 *
 * Arguments:
 *      void * unusedpointer: currently unused.
 *      unsigned long origin: vehicle arrival Direction
 *
 * Returns:
 *      nothing.
 *
 * Notes:
 *      Each simulation thread runs this function
 *
 */

static
void
vehicle_simulation(void * unusedpointer, 
               unsigned long thread_num)
{
  int i;
  Vehicle v;
  time_t before_sec, after_sec, wait_sec;
  uint32_t before_nsec, after_nsec, wait_nsec;
  int sleeptime;
  /* avoid unused variable warnings. */
  (void) unusedpointer;

  KASSERT((long)thread_num < NumThreads);
  for(i=0;i<NumIterations;i++) {

    /* mix things up a bit: actual interarrival time is
       InterArrivalTime +/- 1 */
    sleeptime = InterArrivalTime + random()%3 - 1;
    KASSERT(sleeptime >= InterArrivalTime-1);
    KASSERT(sleeptime <= InterArrivalTime+1);
    /* wait for the next vehicle to arrive */
    clocknap(sleeptime);
    /* try to mix things up a bit more */
    if (random()%NumThreads  < thread_num) {
      thread_yield();
    }
    
    /* choose where this vehicle is coming from */
    v.origin = choose_direction();
    /* choose where this vehicle is heading */
    v.destination = v.origin + (random()%3) + 1;
    if (v.destination >= 4) {
      v.destination = v.destination % 4;
    }
    KASSERT(4 > v.origin);
    KASSERT(4 > v.destination);
    KASSERT(v.origin != v.destination);

    /* invoke entry synchronization function */
    /* this should block until it is OK for this vehicle to
       enter the intersection */
    /* we also measure the time spent blocked */
    gettime(&before_sec,&before_nsec);
    intersection_before_entry(v.origin, v.destination);
    gettime(&after_sec,&after_nsec);

    /* enter the intersection */
    /* note: we are setting a global pointer to point to a local
       structure (v) here.   In general, this is dangerous.
       However, we will be unsetting the 
       pointer before the local structure goes out of scope */
    P(mutex);
    KASSERT(vehicles[thread_num] == NULL);
    vehicles[thread_num] = &v;

    /* check to make sure that the synchronization constraints have
       not been violated */
    /* this call will panic if a constraint has been violated */
    check_constraints(thread_num);
    V(mutex);

    /* spend some time in the intersection */
    in_intersection();

    /* leave the intersection */
    /* here, we are unsetting the global pointer */
    P(mutex);
    KASSERT(vehicles[thread_num] == &v);
    vehicles[thread_num] = NULL;
    V(mutex);

    /* invoke synchronization exit function */
    intersection_after_exit(v.origin, v.destination);

    getinterval(before_sec,before_nsec,after_sec,after_nsec,&wait_sec,&wait_nsec);
    P(perf_mutex);
    total_wait_secs[v.origin] += wait_sec;
    total_wait_nsecs[v.origin] += wait_nsec;
    if (total_wait_nsecs[v.origin] > 1000000000) {
      total_wait_nsecs[v.origin] -= 1000000000;
      total_wait_secs[v.origin] ++;
    }
    wait_count[v.origin]++;
    if (wait_sec > max_wait_secs[v.origin]) {
      max_wait_secs[v.origin] = wait_sec;
      max_wait_nsecs[v.origin] = wait_nsec;
    }
    else if ((wait_sec == max_wait_secs[v.origin])&&
	     (wait_nsec > max_wait_nsecs[v.origin])) {
      max_wait_nsecs[v.origin] = wait_nsec;
    }
    V(perf_mutex);
  }

  /* indicate that this simulation is finished */
  V(SimulationWait); 
}


/*
 * traffic_simulation()
 *
 * Arguments:
 *      int nargs: should be 1 or 5
 *      char ** args: args[1] = number of threads
 *                    args[2] = number of iterations per thread
 *                    args[3] = vehicle interarrival time (seconds)
 *                    args[4] = vehicle intersection time (seconds)
 *                    args[5] = direction bias (0=unbiased, 1=biased)
 *
 * Returns:
 *      0 on success, otherwise an error code from kern/errno.h
 *
 * Notes:
 *      Driver code to start up simulation threads and wait for them
 *      to finish
 */

int
traffic_simulation(int nargs,
		  char ** args)
{
  int i;
  int error;

  if ((nargs != 1) && (nargs != 6)) {
    kprintf("Usage: <command> [threads iterations interarrivaltime servicetime directionbias\n");
    return EINVAL;  // return failure indication
  }

  if (nargs == 6) {
    NumThreads = atoi(args[1]);
    if (NumThreads <= 0 || NumThreads > MAX_THREADS) {
      kprintf("invalid number of threads: %d\n",NumThreads);
      return EINVAL;
    }
    NumIterations = atoi(args[2]);
    if (NumIterations < 0) {
      kprintf("invalid number of iterations per thread: %d\n",NumIterations);
      return EINVAL;
    }
    InterArrivalTime = atoi(args[3]);
    if (InterArrivalTime < 0) {
      kprintf("invalid interarrival time: %d\n",InterArrivalTime);
      return EINVAL;
    }
    ServiceTime = atoi(args[4]);
    if (ServiceTime < 0) {
      kprintf("invalid service time: %d\n",ServiceTime);
      return EINVAL;
    }
    DirectionBias = atoi(args[5]);
    if ((DirectionBias != 0) && (DirectionBias != 1)) {
      kprintf("Direction Bias (%d) must be 0 (unbiased) or 1 (biased)\n",DirectionBias);
      return EINVAL;
    }
  }
  
  kprintf("Threads: %d Iterations: %d Interarrival time: %d  Service time: %d  Bias?: %s\n",
          NumThreads,NumIterations,InterArrivalTime,ServiceTime,DirectionBias?"yes":"no");

  /* initialize our simulation state */
  initialize_state();

  /* get simulation start time */
  gettime(&start_sec,&start_nsec);

  for (i = 0; i < NumThreads; i++) {
    error = thread_fork("vehicle_simulation thread", NULL, vehicle_simulation, NULL, i);
    if (error) {
      panic("traffic_simulation: thread_fork failed: %s\n", strerror(error));
    }
  }
  
  /* wait for all of the vehicle simulations to finish before terminating */  
  for(i=0;i<NumThreads;i++) {
    P(SimulationWait);
  }

  /* get simulation end time */
  gettime(&end_sec,&end_nsec);

  /* clean up the simulation state */
  cleanup_state();

  /* display performance stats */
  print_perf_stats();

  return 0;
}

/*
 * End of traffic.c
 */
