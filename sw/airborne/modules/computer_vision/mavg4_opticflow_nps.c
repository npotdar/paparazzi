/*
 * Optical flow calculations for the Bebop
 *
 */
#include "mavg4_opticflow.h"

#include <stdio.h>
#include <pthread.h>

/* ### Defaults defined for opticalflow SEE .H FILE! ###*/
#define OPTICFLOW_MAX_TRACK_CORNERS 30
#define OPTICFLOW_WINDOW_SIZE 20
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#define OPTICFLOW_MAX_ITERATIONS 10
#define OPTICFLOW_THRESHOLD_VEC 2
#define OPTICFLOW_FAST9_ADAPTIVE FALSE
#define OPTICFLOW_FAST9_THRESHOLD 11
#define OPTICFLOW_FAST9_MIN_DISTANCE 5

/* Optical flow and processing variables */
#define OPTICFLOW_PADDING 50,70	// (int) pad width [px], (int) pad height [px] | Pad image on left, right, top, bottom to not scan!
#define PAD_SORT 50
#define OPTICFLOW_PROCESS_SIZE 272,272			// (int) width [px], (int) height [px] | Processed image size 272,272
#define OPTICFLOW_SORT 272

#define SEGMENT_AMOUNT 5						// (int) segments | Number of semgnets to make

/* Debugging */
#define VIDEO_SIZE OPTICFLOW_PROCESS_SIZE				// 272,272


/* ### Global data used for obstacle avoidance ### */
uint8_t TRANS_MOVE = TRUE;							// (bool) | Set to false when not translating to avoidance false positive

float DETECT_THRESHOLD = 100;							// (float) | Threshold for depth reciprocal
float OBS_HEADING_SET = 60.0;						// (float) | Heading change on detection

uint8_t OBS_DETECT = FALSE;							// (bool) | Obstacle detected?
float OBS_HEADING = 60;								// (float) | Obstacle heaing change

uint8_t ERROR_COUNT = 0;							// (int) | Image Error counter
float IMGERROR_THRESHOLD = 0.0;							// (float) | Image Error threshold
float ERROR_ADD = 0.0;								// (float) | Image Error addition image difference
float ERROR_AVG = 0.0;								// (float) | Average image error over 4 images


/* ### Storage variables ### */

struct opticflow_t opticflow;						// Opticflow calculations

/************************************************************
 * INITIALISE OPTICAL FLOW
 ************************************************************/

/**
 * Initialize values for optical flow DUMMY
 */
void opticflow_module_init(void){

	struct opticflow_t *opticflowp = &opticflow;

	opticflowp->got_first_img = FALSE;

	// Set the default values */
	opticflowp->max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
	opticflowp->window_size = OPTICFLOW_WINDOW_SIZE;
	opticflowp->subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
	opticflowp->max_iterations = OPTICFLOW_MAX_ITERATIONS;
	opticflowp->threshold_vec = OPTICFLOW_THRESHOLD_VEC;

	opticflowp->fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
	opticflowp->fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
	opticflowp->fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;
}


/****************************************************************
 * OPTICAL FLOW THREAD
 ****************************************************************/

void opticflow_module_run(void){}		
void *opticflow_module_calc(struct opticflow_t *opticflow, struct image_t *img, struct opticflow_result_t *result){return 0;}

/****************************************************************
 * OPTICAL FLOW FUNCTION
 ****************************************************************/

void opticflow_calc_frame(struct opticflow_t *opticflow, struct image_t *img, struct opticflow_result_t *result){}

/****************************************************************
 * MODULE START STOP CONTROL
 ****************************************************************/

void opticflow_module_start(void){}
void opticflow_module_stop(void){}
/****************************************************************
 * OBSTACLE CONTROL
 ****************************************************************/
/*
 *  Send heading change if OBS_DETECT is true
 */
float obs_heading(){
	float heading_change;
	if(OBS_DETECT){
		OBS_DETECT = FALSE;
		heading_change = OBS_HEADING;
		OBS_HEADING = 0;
		printf("heading change in block is %f\n",heading_change);
		//return heading_change;
		return heading_change;
	} else {
		printf("wrong block");
		return 0;
	}
}

uint8_t trans_false(){
	TRANS_MOVE = FALSE;
	return FALSE;
}

uint8_t trans_true(){
	TRANS_MOVE = TRUE;
	OBS_DETECT = FALSE;
	return FALSE;
}

