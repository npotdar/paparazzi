#ifndef FLOW_NAVIGATION_H
#define FLOW_NAVIGATION_H
#include <inttypes.h>


//extern int32_t incrementForAvoidance;
//extern uint8_t safeToGoForwards;
extern float incrx;
extern float incry;
extern float distthresh;
extern float wallscale;
//extern float objectscale;
//extern float objectdet;
//extern uint8_t objectDetected;
extern float waythresh;


extern void flow_navigation_init(void);
//extern uint8_t flow_test(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t changeHeading(void);
extern uint8_t chooseRandomIncrementAvoidance(void);

extern void checkDistance(void);
extern uint8_t distToLine(void);
extern uint8_t minDistanceExceed(void);
extern uint8_t minDistancePass(void);
extern uint8_t initialiseLines(uint8_t wp_1, uint8_t wp_2,uint8_t wp_3, uint8_t wp_4,uint8_t wp_targetfun, uint8_t wp_headingfun);



#endif
