#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/* Array that lists each direction at the index that corresponds to the right turn */
enum Directions rightTurn[4] = {west, north, east, south};

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
//static struct semaphore *intersectionSem;
static struct lock *intersectionLock;
static struct cv *intersectionCV;
static struct carList *listHead;
static int totalIntersectionCars;


/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */

  // intersectionSem = sem_create("intersectionSem",1);
  // if (intersectionSem == NULL) {
  //   panic("could not create intersection semaphore");
  // }
  // return;

// 1. Create a lock and a conditional variable
// 2. The linkedlist head will be declared global. Here initialize to NULL
// 3. Total number of cars in intersection will be global. Here initialize to zero

    intersectionLock = lock_create("intersectionLock");
    if (intersectionLock == NULL) {
        panic("could not create intersection lock");
    }

    intersectionCV = cv_create("intersectionCV");
    if (intersectionCV == NULL) {
        panic("could not create intersection CV");
    }

    listHead = NULL;
    totalIntersectionCars = 0;

    return;
}



/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  // KASSERT(intersectionSem != NULL);
  // sem_destroy(intersectionSem);


  // 1. Destroy the lock and conditional variable
    KASSERT(intersectionLock != NULL);
    KASSERT(intersectionCV != NULL);

    lock_destroy(intersectionLock);
    cv_destroy(intersectionCV);

    return;
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  // (void)origin;  /* avoid compiler complaint about unused parameter */
  // (void)destination;  avoid compiler complaint about unused parameter 
  // KASSERT(intersectionSem != NULL);
  // P(intersectionSem);



//1. Acquire the lock
//2. Check if for each car in the linked list the 3 conditions are being met with the new car
//3. If a car already in the linked list does not comply with any condition with the new car: Call cv_wait
//4. Else add the new car to the linked list since there is no conflict
//5. Release the lock
// 3.1 When return from cv_wait check again in the loop if all conditions are being met with the existing cars

    KASSERT(intersectionLock != NULL);
    KASSERT(intersectionCV != NULL);

    struct carList *newCar = kmalloc(sizeof(struct carList));
    if (newCar == NULL) {
        panic("Could not create a new car");
    }

    newCar->origin = origin;
    newCar->destination = destination;
    newCar->Next = NULL;

    lock_acquire(intersectionLock);

    /* No cars in the intersection yet */
    if (listHead == NULL)
    {
        listHead = newCar;
        totalIntersectionCars += 1;
    }
    /* Cars in the intersection exist already */
    else
    {
        struct carList *tempPtr = listHead;

        while (tempPtr != NULL)
        {
            if ((tempPtr->origin == origin) || ((tempPtr->origin == destination) && (tempPtr->destination == origin)) ||
                ((tempPtr->destination != destination) && ((destination == rightTurn[origin]) || (tempPtr->destination == 
                 rightTurn[tempPtr->origin]))))
            {
                tempPtr = tempPtr->Next;
            }
            else
            {
                cv_wait(intersectionCV, intersectionLock);
                tempPtr = listHead;
            }
        }

        // Add the new car to the linked list here
        newCar->Next = listHead;
        listHead = newCar;
        totalIntersectionCars += 1;
    }

    lock_release(intersectionLock);
    return;
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  //(void)origin;  /* avoid compiler complaint about unused parameter */
  //(void)destination; /* avoid compiler complaint about unused parameter */
  //KASSERT(intersectionSem != NULL);
  //V(intersectionSem);

  // 1. Acquire lock
  // 2. Remove this car from the linked list
  // 3. Broadcast CV signal to wake up all waiting cars
  // 4. Release the lock

    KASSERT(intersectionLock != NULL);
    KASSERT(intersectionCV != NULL);

    lock_acquire(intersectionLock);

    struct carList *tempPtrFront = listHead->Next;
    struct carList *tempPtrBack = listHead;

    /* If the car is on the head remove it right away */

    if (totalIntersectionCars == 1 && ((listHead->origin != origin) && (listHead->destination != destination)))
    {
        panic("A car supposed to be at head is not here!!!");
    }

    if ((listHead->origin == origin) && (listHead->destination == destination))
    {
        listHead = tempPtrFront;
        kfree(tempPtrBack);
        totalIntersectionCars -= 1;

        cv_broadcast(intersectionCV, intersectionLock);
        lock_release(intersectionLock);
        return;
    }
    
    /* Look for the curent car in the list */
    while(tempPtrFront != NULL)
    {
        if ((tempPtrFront->origin == origin) && (tempPtrFront->destination == destination))
        {
            break;
        }

        tempPtrBack = tempPtrFront;
        tempPtrFront = tempPtrFront->Next;
    }

    if (tempPtrFront == NULL)
    {
        lock_release(intersectionLock);
        panic("A car supposed to be in the list is not here!!!");
    }

    /* Remove the car from the list */
    tempPtrBack->Next = tempPtrFront->Next;
    kfree(tempPtrFront);
    totalIntersectionCars -= 1;

    /* Broadcast to all waiting cars to check if they can enter the intersection */
    cv_broadcast(intersectionCV, intersectionLock);
    lock_release(intersectionLock);
    return;
}
