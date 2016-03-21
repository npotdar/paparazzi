#include "modules/testg4_module/flow_navigation.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/navigation/waypoints.h"
#include "state.h"
#include <stdlib.h>
#include <math.h>

int32_t incrementForAvoidance;
uint8_t safeToGoForwards;
struct Line{
	float a;
	float b;
	float c;
};


struct Line constructLine(uint8_t wp_1,uint8_t wp_2){
	struct Line line;
	float p1x = waypoint_get_x(wp_1);
	float p1y = waypoint_get_y(wp_1);
	float p2x = waypoint_get_x(wp_2);
	float p2y = waypoint_get_y(wp_2);
	printf("p1x,p1y,p2x,p2y are %f,%f,%f,%f\n",p1x,p1y,p2x,p2y);
	line.a = p1y-p2y;
	line.b = p2x-p1x;
	line.c = p1x*p2y - p2x*p1y;
	printf("a,b,c are %f,%f,%f\n",line.a,line.b,line.c);
	return line;
}

void flow_navigation_init() {
	incrementForAvoidance = 360;
	safeToGoForwards = TRUE;
}

float absol(float in){
	if (in>0){
		return in;
	}
	else{
		return in*-1;
	}
}

/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}

//uint8_t testfun(uint8_t wp_id){
//	printf("WP ID is %u, x is %f\n",wp_id,waypoint_get_x(wp_id));
//	return FALSE;
//}

uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters){
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  printf("heading is is: %ld, sin = %f, cos = %f\n",nav_heading,sin_heading,cos_heading);
	  printf("distanceMeters = %f, x+ = %f, y+ = %f\n", distanceMeters,POS_BFP_OF_REAL(sin_heading * (distanceMeters)),POS_BFP_OF_REAL(cos_heading * (distanceMeters)));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same
	  printf("x is: %ld\n",new_coor.x);
	  printf("y is: %ld\n",new_coor.y);

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t chooseRandomIncrementAvoidance(){

	int r = rand() % 2;
	if(r==0){
		incrementForAvoidance=350;
	}
	else{
		incrementForAvoidance=-350;
	}
	return FALSE;
}



/**
 * Compute the distance between yourself (coordinates xself,yself) and a line given by y=ax+b
 */
uint8_t distToLine(uint8_t wp_1, uint8_t wp_2){
	struct Line line = constructLine(wp_1,wp_2);
	float a = line.a;
	float b = line.b;
	float c = line.c;
	struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
	float xself = (float)pos->x;
	float yself = (float)pos->y;
	float distance = absol(a*xself+b*yself+c)/sqrt(a*a+b*b);
	printf("distance is %f\n",distance);
	return FALSE;
}


