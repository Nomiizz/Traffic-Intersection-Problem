# Traffic Intersection Problem

In the traffic intersection problem, vehicles are trying to pass through an intersection of two roads, one
north/south, the other east/west, without colliding. Each vehicle arrives at the intersection from one of four
directions (north, south, east, or west), called it’s origin. It is trying to pass through the intersection and
exit in some direction other than its origin, called it’s destination.

Each arriving vehicle is simulated by a thread, and the intersection is a critical section. Each vehicle (thread) enters the intersection (critical section) and eventually leaves. The vehicle simulation works by creating a fixed number of concurrent threads. Each thread simulates a sequence of vehicles attempting to pass through the intersection. In a loop, each thread generates a random vehicle (randomly choosing an origin and destination for the vehicle) and then enters and leaves the critical section to simulate the vehicle passing through the intersection. Threads then sleep for a configurable period of time before generating another random vehicle and repeating the process.

The solution in 'traffic_synch.c' uses a conditional variable that provides mutual exclusion to a linked list of cars in the intersecttion (critical section). The solution ensures maximum efficiency (waiting time on average for each car is minimum) and maximum fairness (no particular car should wait too long).