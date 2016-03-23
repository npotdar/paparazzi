#include "modules/testg4_module/flow_navigation.h"
#include "object_input.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/navigation/waypoints.h"
#include "state.h"
#include <stdlib.h>
#include <math.h>

int32_t incrementForAvoidance;
uint8_t safeToGoForwards;
float incrx = 0.0;
float incry = 0.3;
float distthresh = 0.8;
float wallscale = 0.1;
float objectscale = -0.01745;
float objectdet = 0;
float Vx = 0;
float Vy = 1;
float yxratio = 1;
uint8_t wp_target;
uint8_t wp_heading;

struct Line{
	float a;
	float b;
	float c;
};


struct Line lines[4];

//float objTurnCommand(void){
//	return objectdet;
//}

struct Line constructLine(uint8_t wp_1,uint8_t wp_2){
	struct Line line;
	float p1x = waypoint_get_x(wp_1);
	float p1y = waypoint_get_y(wp_1);
	float p2x = waypoint_get_x(wp_2);
	float p2y = waypoint_get_y(wp_2);
	line.a = p1y-p2y;
	line.b = p2x-p1x;
	line.c = p1x*p2y - p2x*p1y;
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
void distToLine(){
	/*
	struct EnuCoor_f *speed = stateGetSpeedEnu_f();
	printf("speed x,y,z = %f,%f,%f\n",speed->x,speed->y,speed->z);
	speed->y=1;
	printf("speed x2,y2,z2 = %f,%f,%f\n",speed->x,speed->y,speed->z);
	stateSetSpeedEnu_f(speed);
	printf("speed x3,y3,z3 = %f,%f,%f\n",speed->x,speed->y,speed->z);
	*/
	struct EnuCoor_f *pos = stateGetPositionEnu_f(); // Get your current position
	float xself = (float)pos->x;
	float yself = (float)pos->y;
	for(int i=0;i<4;i = i+1){
		float a = lines[i].a;
		float b = lines[i].b;
		float c = lines[i].c;
		float distance = absol(a*xself+b*yself+c)/sqrt(a*a+b*b);
		float closestx = (b*(b*xself-a*yself)-a*c)/(a*a+b*b);
		float closesty = (a*(-b*xself+a*yself)-b*c)/(a*a+b*b);
		float xinc = xself-closestx;
		float yinc = yself-closesty;
		float norm = sqrt((xinc*xinc+yinc*yinc));
		xinc = xinc/norm;
		yinc = yinc/norm;
		if(distance<distthresh ){
			incrx = incrx + wallscale*xinc*(1/(pow(distance,1.6)));
			incry = incry + wallscale*yinc*(1/(pow(distance,1.6)));
		}
	}
	float objcommand = objTurnCommand();
	float incrhead = objectdet*objectscale;
	float sin_incrhead = sinf(incrhead);
	float cos_incrhead = cosf(incrhead);
	float xprime = incrx*cos_incrhead - incry*sin_incrhead;
	float yprime = incrx*sin_incrhead + incry*cos_incrhead;
	incrx = xprime;
	incry = yprime;
	yxratio = incry/incrx;
	float incrheadx = incrx*10;
	float incrheady = incry*10;
	//printf("incrheadx,incrheady is: %f,%f\n",incrheadx,incrheady);
	float wayheadx = xself + incrheadx;
	float wayheady =yself + incrheady;
	waypoint_set_xy_i(wp_heading,POS_BFP_OF_REAL(wayheadx),POS_BFP_OF_REAL(wayheady));
	if(absol(incrx)>absol(incry)){
		incrx = capFun(incrx,0.5,-0.5);
		incry = incrx*yxratio;
	}
	else{
		incry = capFun(incry,0.5,-0.5);
		incrx = incry/yxratio;
	}
	//printf("incrx,incry is: %f,%f\n",incrx,incry);
	float wayx = xself+incrx;
	float wayy = yself+incry;

	waypoint_set_xy_i(wp_target,POS_BFP_OF_REAL(wayx),POS_BFP_OF_REAL(wayy));
	bool_t temp = nav_set_heading_towards_waypoint(wp_heading);
}

uint8_t initialiseLines(uint8_t wp_1, uint8_t wp_2,uint8_t wp_3, uint8_t wp_4,uint8_t wp_targetfun, uint8_t wp_headingfun){
	lines[0]=constructLine(wp_1,wp_2);
	lines[1] = constructLine(wp_2,wp_3);
	lines[2] = constructLine(wp_3,wp_4);
	lines[3] = constructLine(wp_4,wp_1);
	wp_target = wp_targetfun;
	wp_heading = wp_headingfun;
	return FALSE;
}



