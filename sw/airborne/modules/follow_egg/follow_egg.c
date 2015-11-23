/*
 * Copyright (C) Roland and Sjoerd
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_egg/follow_egg.c"
 * @author Roland and Sjoerd
 * follows the egg next to the mavlab
 */

#include "modules/follow_egg/follow_egg.h"
#include "modules/stereo_cam/stereocam.h"
#include "generated/flight_plan.h"

#include "subsystems/abi.h"
#ifndef VISION_VELOCITY_ESTIMATE_ID
#define VISION_VELOCITY_ESTIMATE_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VISION_VELOCITY_ESTIMATE_ID)
static abi_event velocity_est_ev;
#define M_PI 3.14159265358979323846
#include "guidance.h"
// Paparazzi Data
#include "state.h"

#include "subsystems/datalink/telemetry.h"
// Interact with navigation
#include "navigation.h"
#include "modules/computer_vision/opticflow_module.h"

float lastKnownVelX;
float lastKnownVelY;

static void new_velocity_estimate(uint8_t sender_id __attribute__((unused)),
    uint32_t stamp, float vel_x, float vel_y, float vel_z, float noise)
{
	lastKnownVelX= vel_x;
	lastKnownVelY= vel_y;
}
void follow_egg_init() {
	printf("Egg follow init");
	 AbiBindMsgVELOCITY_ESTIMATE(VISION_VELOCITY_ESTIMATE_ID, &velocity_est_ev, new_velocity_estimate);

}
int32_t latestVelRefX=0;
int32_t latestVelRefY=0;
int setVelocityReference(float forward, float sideways){
	printf("Setting velocity reference\n");
	float newHeading =stateGetNedToBodyEulers_f()->psi;
	nav_set_heading_rad(newHeading);

	float sin_heading_forward = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	float cos_heading_forward = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
	int newPosX = POS_BFP_OF_REAL(sin_heading_forward * forward);
	int newPosY = POS_BFP_OF_REAL(cos_heading_forward * forward);
//	float sin_heading_sideways = sinf(ANGLE_FLOAT_OF_BFP(nav_heading)+M_PI/2);
//	float cos_heading_sideways = cosf(ANGLE_FLOAT_OF_BFP(nav_heading)+M_PI/2);
//	newPosX+=POS_BFP_OF_REAL(sin_heading_sideways * sideways);
//	newPosY+=POS_BFP_OF_REAL(cos_heading_sideways * sideways);
	navigation_carrot.x=newPosX;
	navigation_carrot.y=newPosY;
	latestVelRefX=newPosX;
	latestVelRefY=newPosY;
	return 1;
}
void set_heading_following_egg() {
	  nav_altitude = waypoints[WP_p1].enu_i.z;
	  waypoints[WP_p1].x = stateGetPositionEnu_f()->x; \
	     waypoints[WP_p1].y = stateGetPositionEnu_f()->y; \
	     waypoints[WP_p1].a = stateGetPositionEnu_f()->z + ground_alt; \
//	navigation_carrot.x=latestVelRefX;
//	navigation_carrot.y=latestVelRefY;
	if(stereocam_data.fresh){
		printf("Set heading Egg follow");
		stereocam_data.fresh=0;

	    int far_away_threshold = 18;
	    int x = 5;
	    uint8_t valueClosest=0;
	    for(x=5; x < 120; x++){
	    	if(stereocam_data.data[x] < far_away_threshold &&stereocam_data.data[x]>0 ){
	    		valueClosest=stereocam_data.data[x];
	    		break;
	    	}
	    }
	    int indexRight;
	    int highValuesRightCount=0;
	    for(indexRight=40; indexRight < 120; indexRight++){
			if(stereocam_data.data[indexRight] > 60){
				highValuesRightCount++;
			}
		}
	    /*
	    if(highValuesRightCount>0){
	    	follow_egg_roll=-3.0;
	    }
	    else{
	    	follow_egg_roll=0.0;
	    }*/

	    int indexAll=0;
	    int totalSum=0;
	    int totalSumCount=0;
	    for(indexAll=0; indexAll < 120; indexAll++){
	    	if(stereocam_data.data[indexAll]>5 && stereocam_data.data[indexAll] < 110){
	    		totalSum+=stereocam_data.data[indexAll];
	    		totalSumCount++;
	    	}
		}
	    float averageClose=0.0;
	    if(totalSumCount>0){
	      averageClose=(totalSum/totalSumCount);
	    }

		/*if(averageClose>50){
			follow_egg_pitch=4.0;
		}
		else{

			follow_egg_pitch=-1.0;
		}*/
	    uint8_t toSend=x;

	    float heading_change = (float) (x-55.0)*0.012; // convert pixel location to radians
	    DOWNLINK_SEND_FOLLOWEGG(DefaultChannel, DefaultDevice, &toSend, &heading_change,&highValuesRightCount,&averageClose);

		float newHeading =stateGetNedToBodyEulers_f()->psi+heading_change;
		nav_set_heading_rad(newHeading);



	}
}

