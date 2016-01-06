/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_me/follow_me.c"
 * @author Roland
 * follows a person on the stereo histogram image.
 * It searches for the highest peak and adjusts its roll and pitch to hover at a nice distance.
 */

#include "modules/stereocam/stereocam_forward_velocity/stereocam_forward_velocity.h"
#include "modules/stereocam/stereocam.h"
#include "state.h"
#include "navigation.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
#include "generated/flight_plan.h"

#define AVERAGE_VELOCITY 0
#define DANGEROUS_CLOSE_DISPARITY 40
#define CLOSE_DISPARITY 33
#define DANGEROUS_CLOSE_DISTANCE 1.0
#define CLOSE_DISTANCE 2.0
#define LOW_AMOUNT_PIXELS_IN_DROPLET 30


float ref_pitch=0.0;
float ref_roll=0.0;

 struct Gains{
	 float pGain;
	 float dGain;
	 float iGain;
 };
typedef struct Gains gains;

gains stabilisationLateralGains;
gains forwardLateralGains;

int turnFactors[]={300,300,300,200,100,100,100,100};
int countFactorsTurning=6;
int indexTurnFactors=0;

// Forward velocity estimation
#define LENGTH_VELOCITY_HISTORY 6
int disparity_velocity_step = 0;
int disparity_velocity_max_time = 500;
int distancesRecorded = 0;
int timeStepsRecorded = 0;
int velocity_disparity_outliers = 0;
float distancesHistory[500];
float timeStepHistory[500];
float velocityHistory[LENGTH_VELOCITY_HISTORY];
int indexVelocityHistory=0;
float sumVelocities=0.0;

float sumHorizontalVelocities=0.0;

typedef enum{GO_FORWARD,STABILISE,TURN,INIT_FORWARD} avoidance_phase;
avoidance_phase current_state=GO_FORWARD;
int totalStabiliseStateCount = 0;
int totalTurningSeenNothing=0;
float previousLateralSpeed = 0.0;
uint8_t detectedWall=0;
float velocityAverageAlpha = 0.65;
float previousHorizontalVelocity = 0.0;

float ref_disparity_to_keep=40.0;
float pitch_compensation = 0.0;
float roll_compensation=0.0;
int initFastForwardCount = 0;
int goForwardXStages=3;
int counterStab=0;
float previousStabRoll=0.0;
float ref_alt=1.0;
typedef enum{USE_DROPLET,USE_CLOSEST_DISPARITY} avoid_strategy_type;
avoid_strategy_type avoidStrategy = USE_DROPLET;
float headingStereocamStab=0.0;
float someGainWhenDoingNothing=0.0;

float somePitchGainWhenDoingNothing=0.0;
float previousStabPitch=0.0;
int stabPositionCount=0;
void stereocam_forward_velocity_init()
{
	stabilisationLateralGains.pGain=0.6;
	stabilisationLateralGains.dGain=0.05;
	stabilisationLateralGains.iGain=0.01;
	forwardLateralGains.pGain=0.6;
}

void array_pop(float *array, int lengthArray);
void array_pop(float *array, int lengthArray)
{
  int index;
  for (index = 1; index < lengthArray; index++) {
    array[index - 1] = array[index];
  }
}

float calculateForwardVelocity(float distance,float alpha,int MAX_SUBSEQUENT_OUTLIERS,int n_steps_velocity);
float calculateForwardVelocity(float distance,float alpha,int MAX_SUBSEQUENT_OUTLIERS,int n_steps_velocity)
{
	    disparity_velocity_step += 1;
	    float new_dist = 0.0;
	    if (distancesRecorded > 0) {
	      new_dist = alpha * distancesHistory[distancesRecorded - 1] + (1 - alpha) * distance;
	    }
	    // Deal with outliers:
	    // Single outliers are discarded, while persisting outliers will lead to an array reset:
	    if (distancesRecorded > 0 && fabs(new_dist - distancesHistory[distancesRecorded - 1]) > 0.5) {
	      velocity_disparity_outliers += 1;
	      if (velocity_disparity_outliers >= MAX_SUBSEQUENT_OUTLIERS) {
	        // The drone has probably turned in a new direction
	        distancesHistory[0] = new_dist;
	        distancesRecorded = 1;

	        timeStepHistory[0] = disparity_velocity_step;
	        timeStepsRecorded = 1;
	        velocity_disparity_outliers = 0;
	      }
	    } else {
	        //append
	      velocity_disparity_outliers = 0;
	      timeStepHistory[timeStepsRecorded] = disparity_velocity_step;
	      distancesHistory[distancesRecorded] = new_dist;
	      distancesRecorded++;
	      timeStepsRecorded++;
	    }

	    //determine velocity (very simple method):
	    float velocityFound = 0.0;
	    if (distancesRecorded > n_steps_velocity) {
	      velocityFound = distancesHistory[distancesRecorded - n_steps_velocity] - distancesHistory[distancesRecorded - 1];
	    }
	    // keep maximum array size:
	    if (distancesRecorded > disparity_velocity_max_time) {
	    	array_pop(distancesHistory, disparity_velocity_max_time);
	    }
	    if (timeStepsRecorded > disparity_velocity_max_time) {
	    	array_pop(timeStepHistory, disparity_velocity_max_time);
	    }
	    return velocityFound;
}
void increase_nav_heading(int32_t *headingToChange, int32_t increment);
void increase_nav_heading(int32_t *headingToChange, int32_t increment)
{
  *headingToChange = *headingToChange + increment;
}
void stereocam_forward_velocity_periodic()
{

  if (stereocam_data.fresh && stereocam_data.len>20) {
    stereocam_data.fresh = 0;
	uint8_t closest = stereocam_data.data[4];

	uint8_t disparitiesInDroplet = stereocam_data.data[5];
    int horizontalVelocity = stereocam_data.data[8]-127;
    int upDownVelocity = stereocam_data.data[9] -127;

    float  BASELINE_STEREO_MM = 60.0;
    float BRANDSPUNTSAFSTAND_STEREO = 118.0 * 6.0 * 2.0;
	float dist = 5.0;
	if (closest > 0) {
	  dist = ((BASELINE_STEREO_MM * BRANDSPUNTSAFSTAND_STEREO / (float)closest)) / 1000;
	}
	float velocityFound = calculateForwardVelocity(dist,0.65, 5,5);

    float guidoVelocityHorStereoboard = horizontalVelocity/100.0;
    float guidoVelocityHor = 0.0;

    // Set the velocity to either the average of the last few velocities, or take the current velocity with alpha times the previous one
    guidoVelocityHor = guidoVelocityHorStereoboard*velocityAverageAlpha + (1-velocityAverageAlpha)*previousHorizontalVelocity;
    sumHorizontalVelocities+=guidoVelocityHor;
    previousHorizontalVelocity= guidoVelocityHorStereoboard;
    //float guidoVelocityZ = upDownVelocity/100.0;
	ref_pitch=0.0;
    ref_roll=0.0;
    if(autopilot_mode != AP_MODE_NAV){
    	 ref_alt= -state.ned_pos_f.z;
    	 ref_disparity_to_keep=CLOSE_DISPARITY-5;
    	 current_state=TURN;
    	 headingStereocamStab=ANGLE_FLOAT_OF_BFP(INT32_DEG_OF_RAD(stab_att_sp_euler.psi));
    	 roll_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.phi);
    	 pitch_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.theta);
    }


    float differenceD = guidoVelocityHor -previousLateralSpeed;
    previousLateralSpeed=guidoVelocityHor;
    counterStab++;
    if(current_state==GO_FORWARD){
		ref_pitch=-0.1;
    	if(avoidStrategy==USE_CLOSEST_DISPARITY){
			if(dist < DANGEROUS_CLOSE_DISTANCE){
				ref_pitch=0.2;
				detectedWall=1;
			}
			else if(dist < CLOSE_DISTANCE){
				ref_pitch=0.1;
				detectedWall=1;
			}
    	}
    	else{
    		if(disparitiesInDroplet>LOW_AMOUNT_PIXELS_IN_DROPLET){
    			ref_pitch=0.1;
    			detectedWall=1;
    		}
    	}

		float max_roll=0.25;
		float rollToTake = forwardLateralGains.pGain * guidoVelocityHor;
		rollToTake*=-1;
		if(counterStab%4==0){
			if(rollToTake>max_roll){
				ref_roll=max_roll;
			}
			else if(rollToTake<(-1.0*max_roll)){
				ref_roll=-(1.0*max_roll);
			}
			else{
				ref_roll=rollToTake;
			}
		}
		if(closest < DANGEROUS_CLOSE_DISPARITY && detectedWall&& closest>0){
			totalTurningSeenNothing=0;
			current_state=STABILISE;
			totalStabiliseStateCount=0;
			detectedWall=0;
		}

    }
    else if(current_state==STABILISE){
    	totalStabiliseStateCount++;
    	float stab_pitch_pgain=0.1;
    	float pitchDiff = closest- ref_disparity_to_keep;
    	float pitchToTake = stab_pitch_pgain*pitchDiff;

    	ref_pitch=0.0;
		float max_roll=0.25;
		float rollToTake =stabilisationLateralGains.pGain * guidoVelocityHor;
		rollToTake*=-1;

		if(counterStab%4==0){
			if(rollToTake>max_roll){
				ref_roll=max_roll;
			}
			else if(rollToTake<(-1.0*max_roll)){
				ref_roll=-(1.0*max_roll);
			}
			else{
				ref_roll=rollToTake;
			}

			if(pitchToTake>0.1){
				ref_pitch=0.1;
			}
			else if (pitchToTake<-0.1){
				ref_pitch=-0.1;
			}
			else{
				ref_pitch=pitchToTake;
			}

			previousStabRoll=ref_roll;
			someGainWhenDoingNothing+=0.1*ref_roll;
			somePitchGainWhenDoingNothing+=0.1*ref_pitch;
			previousStabPitch=ref_pitch;



		}
		else{
			ref_pitch=0.5*previousStabPitch;
			ref_roll=0.5*previousStabRoll;
		}

		if(abs(closest- ref_disparity_to_keep)<5){
			stabPositionCount++;
		}
		else{
			stabPositionCount=0;
		}
		if(guidoVelocityHor<0.25 && guidoVelocityHor>-0.25 && totalStabiliseStateCount>10){

			if(stabPositionCount>20){
				current_state=TURN;
				indexTurnFactors=0;
				stabPositionCount=0;
			}
		}
       }
    else if(current_state==TURN){
    	ref_pitch=0.0;
    	ref_roll=0.0;

    	headingStereocamStab += 5.0;
    	  if (headingStereocamStab > 360.0)
    		  headingStereocamStab -= 360.0;
    	indexTurnFactors+=1;
    	if(indexTurnFactors>countFactorsTurning){
    		indexTurnFactors = countFactorsTurning;
    	}
    	if(indexTurnFactors > 3){
    		if(avoidStrategy==USE_CLOSEST_DISPARITY){
    			if(closest<CLOSE_DISPARITY && closest>0){
					totalTurningSeenNothing++;
					if(totalTurningSeenNothing>2){
						current_state=GO_FORWARD;
						detectedWall=0;
					}
    			}
    		}
    		else{
    			if(disparitiesInDroplet<LOW_AMOUNT_PIXELS_IN_DROPLET){
    				totalTurningSeenNothing++;
//					if(totalTurningSeenNothing>2){
						current_state=GO_FORWARD;
						detectedWall=0;
//					}
    			}
    		}
    	}
    	else{
    		totalTurningSeenNothing=0;
    	}

    }
    else{
    	current_state=GO_FORWARD;
    }
    nav_set_heading_deg(headingStereocamStab);

    ref_pitch += pitch_compensation;
    ref_roll += roll_compensation;
    DOWNLINK_SEND_STEREO_VELOCITY(DefaultChannel, DefaultDevice, &closest, &disparitiesInDroplet,&dist, &velocityFound,&guidoVelocityHor,&ref_disparity_to_keep,&current_state,&guidance_h_trim_att_integrator.x);
    DOWNLINK_SEND_REFROLLPITCH(DefaultChannel, DefaultDevice, &somePitchGainWhenDoingNothing,&ref_pitch);
  }
}
