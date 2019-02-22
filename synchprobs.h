#ifndef _SYNCHPROBS_H_
#define _SYNCHPROBS_H_

/* enumerated type, for the road intersection problem */
enum Directions
  { 
    north = 0,
    east = 1,
    south = 2,
    west = 3
  };
typedef enum Directions Direction;


struct carList {

	Direction origin;
	Direction destination;
    struct carList *Next;
};

/* student-implemented functions for the road intersection problem */

void intersection_sync_init(void);
void intersection_sync_cleanup(void);
void intersection_before_entry(Direction origin, Direction destination);
void intersection_after_exit(Direction origin, Direction destination);

/* student-implemented functions for the cat/mouse problem */

void cat_before_eating(unsigned int bowl);
void cat_after_eating(unsigned int bowl);
void mouse_before_eating(unsigned int bowl);
void mouse_after_eating(unsigned int bowl);
void catmouse_sync_init(int bowls);
void catmouse_sync_cleanup(int bowls);
void print_state_on(void);
void print_state_off(void);

#endif /* _SYNCHPROBS_H_ */
