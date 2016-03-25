#ifndef FLOW_NAVIGATION_H
#define FLOW_NAVIGATION_H
#include <inttypes.h>


extern int32_t incrementForAvoidance;
extern uint8_t safeToGoForwards;
extern float incrx;
extern float incry;
extern float distthresh;
extern float wallscale;
extern float objectscale;
extern float objectdet;
extern uint8_t objectDetected;


extern void flow_navigation_init(void);
//extern uint8_t flow_test(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t changeHeading();
extern uint8_t chooseRandomIncrementAvoidance();

#endif
