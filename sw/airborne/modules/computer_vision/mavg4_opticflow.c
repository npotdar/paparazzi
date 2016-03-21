/*
 * Optical flow calculations for the Bebop
 *
 */
#include "mavg4_opticflow.h"

#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "subsystems/abi.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"

#define OPTICFLOW_TELE_ID 1


/* ### Defaults defined for opticalflow SEE .H FILE!!! ###*/
#ifndef OPTICFLOW_MAX_TRACK_CORNERS
#define OPTICFLOW_MAX_TRACK_CORNERS 25
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_TRACK_CORNERS)

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_WINDOW_SIZE)

#ifndef OPTICFLOW_SUBPIXEL_FACTOR
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_SUBPIXEL_FACTOR)

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_ITERATIONS)

#ifndef OPTICFLOW_THRESHOLD_VEC
#define OPTICFLOW_THRESHOLD_VEC 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_THRESHOLD_VEC)

#ifndef OPTICFLOW_FAST9_ADAPTIVE
#define OPTICFLOW_FAST9_ADAPTIVE TRUE
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_ADAPTIVE)

#ifndef OPTICFLOW_FAST9_THRESHOLD
#define OPTICFLOW_FAST9_THRESHOLD 20
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_THRESHOLD)

#ifndef OPTICFLOW_FAST9_MIN_DISTANCE
#define OPTICFLOW_FAST9_MIN_DISTANCE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FAST9_MIN_DISTANCE)


/* ### Defaults defined for bebop ###*/
#define OPTICFLOW_DEVICE /dev/video1				// Path to video device
#define OPTICFLOW_DEVICE_SIZE 1408,2112				// (int) width [px], (int) height [px] | Video device resolution
#define OPTICFLOW_SUBDEV FALSE						// (boolean) | Use SUBDEV video device

/* Video device buffers */
#define OPTICFLOW_DEVICE_BUFFERS 15					// (int) | Video device V4L2 buffers

/* Optical flow variables */
#define OPTICFLOW_PADDING 50,50 				// (int) pad width [px], (int) pad height [px] | Pad image on left, right, top, bottom to not scan!
#define OPTICFLOW_PROCESS_SIZE 272,272			// (int) width [px], (int) height [px] | Processed image size


/* ### Storage variables ### */
struct opticflow_t opticflow;						// Opticflow calculations
static struct opticflow_results_t opticflow_result;	// Opticflow results
static struct v4l2_device *opticflow_dev;			// The opticflow camera V4L2 device
static bool_t opticflow_got_result;                	// When we have an optical flow calculation

/* Threads and mutexes*/
static pthread_t opticflow_calc_thread;            	// The optical flow calculation thread

static pthread_mutex_t opticflow_mutex;            	// Mutex lock for thread safety

/* ### Functions ### */
static void opticflow_module_run(void); 				// Dummy function
static void *opticflow_module_calc(void *data);		// Main optical flow calculation thread
static void opticflow_calc_frame(struct opticflow_t *opticflow, struct image_t *img,
									struct opticflow_result_t *result);
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);		// Calculation of timedifference for FPS
static int cmp_flow(const void *a, const void *b);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
/**
 * Send optical flow telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void opticflow_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&opticflow_mutex);
  pprz_msg_send_OPTIC_FLOW_MAVG4(trans, dev, AC_ID,
                               &opticflow_result.fps, &opticflow_result.corner_cnt,
                               &opticflow_result.tracked_cnt, &opticflow_result.flow_x);
  pthread_mutex_unlock(&opticflow_mutex);
}
#endif

/************************************************************
 * INITIALISE OPTICAL FLOW
 ************************************************************/

/**
 * Initialize the optical flow module for the bebop front camera
 */
void opticflow_module_init(void){
	/* Initialise optical flow calculations */
	// Create the image buffers
	image_create(&opticflow->img_gray, OPTICFLOW_PROCESS_SIZE, IMAGE_GRAYSCALE);
	image_create(&opticflow->prev_img_gray, OPTICFLOW_PROCESS_SIZE, IMAGE_GRAYSCALE);

	//Set the previous values
	opticflow->got_first_img = FALSE;

	// Set the default values */
	opticflow->max_track_corners = OPTICFLOW_MAX_TRACK_CORNERS;
	opticflow->window_size = OPTICFLOW_WINDOW_SIZE;
	opticflow->subpixel_factor = OPTICFLOW_SUBPIXEL_FACTOR;
	opticflow->max_iterations = OPTICFLOW_MAX_ITERATIONS;
	opticflow->threshold_vec = OPTICFLOW_THRESHOLD_VEC;

	opticflow->fast9_adaptive = OPTICFLOW_FAST9_ADAPTIVE;
	opticflow->fast9_threshold = OPTICFLOW_FAST9_THRESHOLD;
	opticflow->fast9_min_distance = OPTICFLOW_FAST9_MIN_DISTANCE;

	opticflow_got_result = FALSE;

	/* Initialise video device settings */
	// Initialise the subdev video device
	if(OPTICFLOW_SUBDEV){
		if (!v4l2_init_subdev("/dev/v4l-subdev1", 0, 1, V4L2_MBUS_FMT_SGBRG10_1X10, OPTICFLOW_DEVICE_SIZE)) {
		      printf("[bebop_front_camera] Could not initialize the v4l-subdev1 subdevice.\n");
		      return;
		    }
	}

	// Initialise the main video device
	opticflow_dev = v4l2_init("/dev/video1", OPTICFLOW_DEVICE_SIZE, OPTICFLOW_DEVICE_BUFFERS, V4L2_PIX_FMT_SGBRG10);

}

/****************************************************************
 * MODULE START STOP CONTROL
 ****************************************************************/

/**
 * Start the optical flow calculation
 */
void opticflow_module_start(void){
	// Check if we not already running
	if (opticflow_calc_thread != 0) {
	    printf("[opticflow_module] Opticflow already started!\n");
	    return;
	  }

	  // Create the opticalflow calculation thread
	  int rc = pthread_create(&opticflow_calc_thread, NULL, opticflow_module_calc, NULL);
	  if (rc) {
	    printf("[opticflow_module] Could not initialize opticflow thread (return code: %d)\n", rc);
	  }
}


/**
 * Stop the optical flow calculation
 */
void opticflow_module_stop(void)
{
  // Stop the capturing
  v4l2_stop_capture(opticflow_dev);

  // Cancel the opticalflow calculation thread
  pthread_cancel(&opticflow_calc_thread);
}


/****************************************************************
 * OPTICAL FLOW THREAD
 ****************************************************************/


/**
 * The main optical flow calculation thread
 * This thread passes the images through the optical flow
 * calculator using FAST corner detection and Lucas Kanade tracking
 */

static void *opticflow_module_calc(void *data __attribute__((unused))){
	// Start the straming on the V4L2 device
	if (!v4l2_start_capture(opticflow_dev)) {
	    printf("[opticflow_module] Could not start capture of the camera\n");
	    return 0;
	  }

	/* Main loop of the optic flow calculation	 */
	while(TRUE){
		/* Define image */
		struct image_t img;
		v4l2_image_get(opticflow_dev, &img);

		/* Do optical flow calculations */
		//Calculate on frame
		struct opticflow_result_t temp_result;
		opticflow_calc_frame(&opticflow, &img, &temp_result);

		//Copy results to shared result memory
		pthread_mutex_lock(&opticflow_mutex);
		memcpy(&opticflow_result, &temp_result, sizeof(struct opticflow_result_t));
		opticflow_got_result = TRUE;
		pthread_mutex_unlock(&opticflow_mutex);

		/* Free image */
		v4l2_image_free(opticflow_dev, &img);
	}
}

/**
 * Run the optical flow on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */

/****************************************************************
 * OPTICAL FLOW FUNCTION
 ****************************************************************/

static void opticflow_calc_frame(struct opticflow_t *opticflow, struct image_t *img,
									struct opticflow_result_t *result){
	// Update FPS for information
	result->fps = 1/(timeval_diff(&opticflow->prev_timestamp,&img->ts) / 1000.);
	memcpy(&opticflow->prev_timestamp,&img->ts,sizeof(struct timeval));

	// Convert image to grayscale
	image_to_grayscale(img, &opticflow->img_gray);

	// Copy to previous image if not set
	if(!opticflow->got_first_img){
		image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
		opticflow->got_first_img = TRUE;
	}

	/* Corner detection */
	// FAST corner detection
	struct point_t *corners = fast9_detect(img, opticflow->fast9_threshold, opticflow->fast9_min_distance,
											OPTICFLOW_PADDING, &result->corner_cnt);

	// FAST Adaptive threshold
	if (opticflow->fast9_adaptive){
		// Decrease and increase the threshold based on previous values TUNABLE!!!
		if (result->corner_cnt < 40 && opticflow->fast9_threshold > 5) {
		  opticflow->fast9_threshold--;
		} else if (result->corner_cnt > 50 && opticflow->fast9_threshold < 60) {
		  opticflow->fast9_threshold++;
		}
	}

	// FAST Check if corners were detected
	if (result->corner_cnt < 1){
		free(corners);
		image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
		return;
	}

	/* Corner tracking */
	// LUCAS-KANADE optical flow
	result->tracked_cnt = result->corner_cnt; 		// Sets tracked number of corners to corners detected by FAST

	struct flow_t *vectors = opticFlowLK(&opticflow->img_gray, &opticflow->prev_img_gray, corners, &result->tracked_cnt,
            opticflow->window_size / 2, opticflow->subpixel_factor, opticflow->max_iterations,
            opticflow->threshold_vec, opticflow->max_track_corners);

	/* Do something with the flow EDIT HERE*/
	// Get the median flow
	  qsort(vectors, result->tracked_cnt, sizeof(struct flow_t), cmp_flow);
	  if (result->tracked_cnt == 0) {
	    // We got no flow
	    result->flow_x = 0;
	    result->flow_y = 0;
	  } else if (result->tracked_cnt > 3) {
	    // Take the average of the 3 median points
	    result->flow_x = vectors[result->tracked_cnt / 2 - 1].flow_x;
	    result->flow_y = vectors[result->tracked_cnt / 2 - 1].flow_y;
	    result->flow_x += vectors[result->tracked_cnt / 2].flow_x;
	    result->flow_y += vectors[result->tracked_cnt / 2].flow_y;
	    result->flow_x += vectors[result->tracked_cnt / 2 + 1].flow_x;
	    result->flow_y += vectors[result->tracked_cnt / 2 + 1].flow_y;
	    result->flow_x /= 3;
	    result->flow_y /= 3;
	  } else {
	    // Take the median point
	    result->flow_x = vectors[result->tracked_cnt / 2].flow_x;
	    result->flow_y = vectors[result->tracked_cnt / 2].flow_y;
	  }
}


/****************************************************************
 * EXTRA FUNCTIONS
 ****************************************************************/

/**
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 * @return The difference in milliseconds
 */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec = (finishtime->tv_sec - starttime->tv_sec) * 1000;
  msec += (finishtime->tv_usec - starttime->tv_usec) / 1000;
  return msec;
}

/**
 * Compare two flow vectors based on flow distance
 * Used for sorting.
 * @param[in] *a The first flow vector (should be vect flow_t)
 * @param[in] *b The second flow vector (should be vect flow_t)
 * @return Negative if b has more flow than a, 0 if the same and positive if a has more flow than b
 */
static int cmp_flow(const void *a, const void *b)
{
  const struct flow_t *a_p = (const struct flow_t *)a;
  const struct flow_t *b_p = (const struct flow_t *)b;
  return (a_p->flow_x * a_p->flow_x + a_p->flow_y * a_p->flow_y) - (b_p->flow_x * b_p->flow_x + b_p->flow_y *
         b_p->flow_y);
}
