#ifndef FLOW_NAVIGATION_H
#define FLOW_NAVIGATION_H
#include <inttypes.h>


extern int32_t incrementForAvoidance;
extern uint8_t safeToGoForwards;

extern void flow_navigation_init(void);
//extern uint8_t flow_test(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
extern uint8_t chooseRandomIncrementAvoidance();

#endif
