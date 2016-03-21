#include "modules/testg4_module/flow_navigation.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/navigation/waypoints.h"
#include "state.h"
#include <stdlib.h>
#include <math.h>

int32_t incrementForAvoidance;
uint8_t safeToGoForwards;
float incrx = 0;
float incry = 0.3;
struct Line{
	float a;
	float b;
	float c;
};
//struct Line line1;
//struct Line line2;
//struct Line line3;
//struct Line line4;

struct Line lines[4];


struct Line constructLine(uint8_t wp_1,uint8_t wp_2){
	struct Line line;
	float p1x = waypoint_get_x(wp_1);
	float p1y = waypoint_get_y(wp_1);
	float p2x = waypoint_get_x(wp_2);
	float p2y = waypoint_get_y(wp_2);
	//printf("p1x,p1y,p2x,p2y are %f,%f,%f,%f\n",p1x,p1y,p2x,p2y);
	line.a = p1y-p2y;
	line.b = p2x-p1x;
	line.c = p1x*p2y - p2x*p1y;
	//printf("a,b,c are %f,%f,%f\n",line.a,line.b,line.c);
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

float capFun(float in, float upBound, float lowBound){
	if(in>upBound){
		return upBound;
	}
	else if(in<lowBound){
		return lowBound;
	}
	else{
		return in;
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
uint8_t distToLine(uint8_t wp_target){
	struct EnuCoor_f *pos = stateGetPositionEnu_f(); // Get your current position
	float xself = (float)pos->x;
	float yself = (float)pos->y;
	for(int i=0;i<4;i = i+1){
		float a = lines[i].a;
		float b = lines[i].b;
		float c = lines[i].c;
		float distance = absol(a*xself+b*yself+c)/sqrt(a*a+b*b);
		printf("i is: %i distance is: %f\n",i,distance);
		float closestx = (b*(b*xself-a*yself)-a*c)/(a*a+b*b);
		float closesty = (a*(-b*xself+a*yself)-b*c)/(a*a+b*b);
		float xinc = xself-closestx;
		float yinc = yself-closesty;
		float norm = sqrt((xinc*xinc+yinc*yinc));
		xinc = xinc/norm;
		yinc = yinc/norm;
		printf("i is: %i, xinc is: %f, yinc is: %f\n",i,xinc,yinc);
		float slow=0.1;
		//float testx = incrx + slow*xinc*(1/(distance));
		//float testy = incry + slow*yinc*(1/(distance));
		//float mag = sqrt(testx*testx+testy*testy);
		if(distance<1.8 ){
			incrx = incrx + slow*xinc*(1/(distance));
			incry = incry + slow*yinc*(1/(distance));
		}
	}
	float wayx = waypoint_get_x(wp_target);
	float wayy = waypoint_get_y(wp_target);
	incrx = capFun(incrx,0.35,-0.35);
	incry = capFun(incry,0.35,-0.35);
	wayx = wayx+incrx;
	wayy = wayy+incry;
	waypoint_set_xy_i(wp_target,POS_BFP_OF_REAL(wayx),POS_BFP_OF_REAL(wayy));
	bool_t temp = nav_set_heading_towards_waypoint(wp_target);
	printf("incrx2 is: %f, incry2 is: %f\n",incrx,incry);
	return FALSE;
}

uint8_t initialiseLines(uint8_t wp_1, uint8_t wp_2,uint8_t wp_3, uint8_t wp_4){
	lines[0]=constructLine(wp_1,wp_2);
	lines[1] = constructLine(wp_2,wp_3);
	lines[2] = constructLine(wp_3,wp_4);
	lines[3] = constructLine(wp_4,wp_1);
	return FALSE;
}



