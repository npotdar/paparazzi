/*
 * Optical flow calculations for the Bebop
 *
 */
#include "mavg4_opticflow.h"

#include <stdio.h>
#include <pthread.h>
//#include "opticflow/inter_thread_data.h"
#include "state.h"
#include "subsystems/abi.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"
#include "lib/vision/bayer.h"

#include "lib/encoding/rtp.h"
#include "udp_socket.h"

// include board for bottom_camera and front_camera on ARDrone2 and Bebop
//#include BOARD_CONFIG

/* ### Defaults defined for opticalflow SEE .H FILE! ###*/
#define OPTICFLOW_MAX_TRACK_CORNERS 30
#define OPTICFLOW_WINDOW_SIZE 20
#define OPTICFLOW_SUBPIXEL_FACTOR 10
#define OPTICFLOW_MAX_ITERATIONS 10
#define OPTICFLOW_THRESHOLD_VEC 2
#define OPTICFLOW_FAST9_ADAPTIVE FALSE
#define OPTICFLOW_FAST9_THRESHOLD 11
#define OPTICFLOW_FAST9_MIN_DISTANCE 5

/* ### Defaults defined for bebop ###*/
#define OPTICFLOW_DEVICE /dev/video1				// Path to video device
#define OPTICFLOW_DEVICE_SIZE 1408,2112				// (int) width [px], (int) height [px] | Video device resolution

/* Video device buffers */
#define OPTICFLOW_DEVICE_BUFFERS 15		// (int) | Video device V4L2 buffers default: 15

/* Optical flow variables */
#define OPTICFLOW_PADDING 50,70	// (int) pad width [px], (int) pad height [px] | Pad image on left, right, top, bottom to not scan!
#define PAD_SORT 50
#define OPTICFLOW_PROCESS_SIZE 272,272			// (int) width [px], (int) height [px] | Processed image size 544, 544
#define OPTICFLOW_SORT 272

#define SEGMENT_AMOUNT 5						// (int) segments | Number of semgnets to make

/* Debugging */
#define PERIODIC_TELEMETRY TRUE
#define OPTICFLOW_DEBUG TRUE
#define VIDEO_SIZE OPTICFLOW_PROCESS_SIZE				// 272,272
#define VIDEO_THREAD_SHOT_PATH "/data/ftp/internal_000/images"

/* ### Storage variables ### */
#if OPTICFLOW_DEBUG
	  struct UdpSocket video_sock;
	  //struct image_t img_jpeg;
#endif

struct opticflow_t opticflow;						// Opticflow calculations
static struct opticflow_result_t opticflow_result;	// Opticflow results
static struct v4l2_device *opticflow_dev;			// The opticflow camera V4L2 device

/* Threads and mutexes*/
static pthread_t opticflow_calc_thread;            	// The optical flow calculation thread

static pthread_mutex_t opticflow_mutex;            	// Mutex lock for thread safety

// Global data used for obstacle avoidance
float DETECT_THRESHOLD = 1;							// (float) | Threshold for depth
float OBS_HEADING_SET = 60.0;						// (float) | Heading change on detection

uint8_t OBS_DETECT = FALSE;							// Obstacle detected?
float OBS_HEADING = 0;								// Obstacle heaing change




/* ### Functions ### */
//static void opticflow_module_run(void); 				// Dummy function
static void *opticflow_module_calc(void *data);		// Main optical flow calculation thread
static int sort_on_x(const void *a, const void *b);
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);		// Calculation of timedifference for FPS
//static int cmp_flow(const void *a, const void *b);
static void video_thread_save_shot(struct image_t *img, struct image_t *img_jpeg, int shot_number);

void opticflow_module_run(void){};

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
  pprz_msg_send_OFG(trans, dev, AC_ID,
                               &opticflow_result.fps, &opticflow_result.corner_cnt,
                               &opticflow_result.tracked_cnt, &opticflow_result.flows, &opticflow_result.obs_detect
							   &opticflow_result.obs_heading);
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
	struct opticflow_t *opticflowp = &opticflow;

	// Create images to be used by FAST and Lucas Kanade
	image_create(&opticflowp->img_gray, OPTICFLOW_PROCESS_SIZE, IMAGE_GRAYSCALE);
	image_create(&opticflowp->prev_img_gray, OPTICFLOW_PROCESS_SIZE, IMAGE_GRAYSCALE);

	//Set the previous values
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

	/* Initialise video device settings */
	// Initialise the main video device
	opticflow_dev = v4l2_init("/dev/video1", OPTICFLOW_DEVICE_SIZE, OPTICFLOW_DEVICE_BUFFERS, V4L2_PIX_FMT_SGBRG10);

	#if PERIODIC_TELEMETRY
	  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OFG, opticflow_telem_send);
	#endif
}




/****************************************************************
 * OPTICAL FLOW THREAD
 ****************************************************************/


/**
 * The main optical flow calculation thread
 * This thread passes the images through the optical flow
 * calculator using FAST corner detection and Lucas Kanade tracking
 */

void *opticflow_module_calc(void *data __attribute__((unused))){

	// Start the straming on the V4L2 device
	if (!v4l2_start_capture(opticflow_dev)) {
	    printf("[opticflow_module] Could not start capture of the camera\n");
	    return 0;
	  }

	// Create the images for streaming purposes only!
	#if OPTICFLOW_DEBUG
	struct image_t img_color;
	image_create(&img_color, VIDEO_SIZE, IMAGE_YUV422);
	struct image_t img_jpeg;
	image_create(&img_jpeg, VIDEO_SIZE, IMAGE_JPEG);
	udp_socket_create(&video_sock, STRINGIFY(BEBOP_FRONT_CAMERA_HOST), 5000, -1, TRUE);
	#endif

	struct image_t img;
	image_create(&img, OPTICFLOW_PROCESS_SIZE, IMAGE_YUV422);

	/* Main loop of the optic flow calculation	 */
	//int counter = 1;
	while(TRUE){
		/* Define image */
		struct image_t img_dev;

		// Get image from v4l2 with native size
		v4l2_image_get(opticflow_dev, &img_dev);
		BayerToYUV(&img_dev, &img, 0, 0);

		//printf("%d, %d\n",img.w, img.h);

		/* Do optical flow calculations */

		//Calculate on frame

		struct opticflow_result_t temp_result;
		opticflow_calc_frame(&opticflow, &img, &temp_result);

		//Copy results to shared result memory
		pthread_mutex_lock(&opticflow_mutex);
		memcpy(&opticflow_result, &temp_result, sizeof(struct opticflow_result_t));
		pthread_mutex_unlock(&opticflow_mutex);


		#if OPTICFLOW_DEBUG
			//BayerToYUV(&img, &img_color, 0, 0);
			//image_copy(&img_ph, &img_color);
			//video_thread_save_shot(&img, &img_jpeg, counter);
			jpeg_encode_image(&img, &img_jpeg, 90, 0);
			rtp_frame_send(
			  &video_sock,           // UDP device
			  &img_jpeg,
			  0,                        // Format 422
			  90, // Jpeg-Quality
			  0,                        // DRI Header
			  0                         // 90kHz time increment
			);
		#endif

		/* Free image */
		v4l2_image_free(opticflow_dev, &img_dev);

		//counter++;
	}

	#if OPTICFLOW_DEBUG
	  image_free(&img_jpeg);
	  image_free(&img_color);
	#endif
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

void opticflow_calc_frame(struct opticflow_t *opticflowin, struct image_t *img,
									struct opticflow_result_t *result){

	//struct image_t img_scaled;
	//image_create(&img_scaled, OPTICFLOW_PROCESS_SIZE,IMAGE_YUV422);
	//image_copy(&img, &img_scaled);

	// Update FPS for information
	result->fps = 1/(timeval_diff(&opticflowin->prev_timestamp,&img->ts) / 1000.);
	memcpy(&opticflowin->prev_timestamp,&img->ts,sizeof(struct timeval));


	// Convert image to grayscale uses PROCESS_SIZE
	image_to_grayscale(img, &opticflowin->img_gray);

	// Copy to previous image if not set
	if(!opticflowin->got_first_img){
		image_copy(&opticflowin->img_gray, &opticflowin->prev_img_gray);
		opticflowin->got_first_img = TRUE;
	}

	// FAST corner detection
	struct point_t *corners = fast9_detect(img, opticflowin->fast9_threshold, opticflowin->fast9_min_distance,
											OPTICFLOW_PADDING, &result->corner_cnt);

	// FAST Adaptive threshold
	if (opticflowin->fast9_adaptive){
		// Decrease and increase the threshold based on previous values TUNABLE!!!
		if (result->corner_cnt < 40 && opticflowin->fast9_threshold > 5) {
		  opticflowin->fast9_threshold--;
		} else if (result->corner_cnt > 50 && opticflowin->fast9_threshold < 60) {
		  opticflowin->fast9_threshold++;
		}
	}

	// FAST Check if corners were detected
	if (result->corner_cnt < 1){
		free(corners);
		image_copy(&opticflowin->img_gray, &opticflowin->prev_img_gray);
		return;
	}
	// LUCAS-KANADE optical flow
	result->tracked_cnt = result->corner_cnt; 		// Sets tracked number of corners to corners detected by FAST

	struct flow_t *vectors = opticFlowLK(&opticflowin->img_gray, &opticflowin->prev_img_gray, corners, &result->tracked_cnt,
            opticflowin->window_size / 2, opticflowin->subpixel_factor, opticflowin->max_iterations,
            opticflowin->threshold_vec, opticflowin->max_track_corners);

	/* ALSO DO COLOUR AVOIDANCE */
	int color_avg_x;
	int color_avg_y;
	int color_count;
	image_yuv422_colorfilt_ext(img,img,&color_count,&color_avg_x,&color_avg_y,0,131,93,255,134,255);
	printf("%d, %d\n", color_avg_x, color_count);

#if OPTICFLOW_DEBUG
	//image_show_points(img, corners, result->corner_cnt);
	image_show_flow(img, vectors, result->tracked_cnt, opticflowin->subpixel_factor);
#endif


	/* Do something with the flow EDIT HERE*/
	//qsort(vectors, result->tracked_cnt, sizeof(struct flow_t), sort_on_x);

	/*BUILD SEGMENTED ARRAY */
	int n;
	int i;
	int fov = 2;
	int iter;
	int magnitude_squared;
	float magnitude_root;

	int segment_size=(OPTICFLOW_SORT - 2*PAD_SORT)/SEGMENT_AMOUNT;
	double segmented_array[SEGMENT_AMOUNT];
	double magnitude_array[SEGMENT_AMOUNT];

	// Construct flow matrix
	for (iter=0; iter<result->tracked_cnt; iter++)
		{
		magnitude_squared = (vectors[i].flow_x * vectors[i].flow_x + vectors[i].flow_y * vectors[i].flow_y );
		magnitude_root = sqrtf(magnitude_squared);
			for (n=0; n<no_segments; n++)
				{
					if (( (vectors[i].pos.x / OPTICFLOW_SUBPIXEL_FACTOR)>(n*segment_size + PAD_SORT) ) && ( (vectors[i].pos.x / OPTICFLOW_SUBPIXEL_FACTOR) <= ((n+1)*segment_size + PAD_SORT) ) )
					{
						magnitude_array[n] = (magnitude_array[n] + (magnitude_root));
					}
				}
		}

	// Construct reciprocal flow matrix
	for (n=0; n<SEGMENT_AMOUNT; n++)
	{
		segmented_array[n]= 1/magnitude_array[n];
	}

#if OPTICFLOW_DEBUG
	for(iter=0;iter < (sizeof (segmented_array) /sizeof (segmented_array[0])); iter++)
		{
			printf("x: %d depth: %f \n",iter+1, segmented_array[iter]);
		}
	printf("\n\n");
#endif

	/*DO OBSTACLE DETECTION AND CONTROL */

	// Check object
	for(i = SEGMENT_AMOUNT/2 - 1 - fov; i < SEGMENT_AMOUNT/2 + fov; i++){
		if (segmented_array[i] < DETECT_THRESHOLD){
			if(i < SEGMENT_AMOUNT/2){
				OBS_DETECT = TRUE;
				OBS_HEADING = OBS_HEADING_SET;
			} else if (i >= SEGMENT_AMOUNT/2){
				OBS_DETECT = TRUE;
				OBS_HEADING = -1*OBS_HEADING_SET;
			}
		}
	}
#if PERIODIC_TELEMETRY
	result->flows = segmented_array;
	result->obs_detect = OBS_DETECT;
	result->obs_heading = OBS_HEADING;
#endif


	/* NEXT LOOP */
	free(corners);
	free(vectors);
	image_switch(&opticflowin->img_gray, &opticflowin->prev_img_gray);
}






/****************************************************************
 * MODULE START STOP CONTROL
 ****************************************************************/

/**
 * Start the optical flow calculation
 */
void opticflow_module_start(void){
	// Check if we are not already running
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
  if(pthread_cancel(opticflow_calc_thread)!=0){
	  printf("Thread killing did not work\n");
  }
}

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
		return heading_change;
	} else {
		return 0;
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

static int sort_on_x(const void *a, const void *b)
{
	const struct flow_t *a_p = (const struct flow_t *)a;
	const struct flow_t *b_p = (const struct flow_t *)b;
	return (a_p->pos.x ) - (b_p->pos.x) ;
}

static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec = (finishtime->tv_sec - starttime->tv_sec) * 1000;
  msec += (finishtime->tv_usec - starttime->tv_usec) / 1000;
  return msec;
}

static void video_thread_save_shot(struct image_t *img, struct image_t *img_jpeg, int shot_number)
{

  // Search for a file where we can write to
  char save_name[128];
  for (; shot_number < 99999; shot_number++) {
    sprintf(save_name, "%s/img_%05d.jpg", VIDEO_THREAD_SHOT_PATH, shot_number);
    // Check if file exists or not
    if (access(save_name, F_OK) == -1) {

      // Create a high quality image (99% JPEG encoded)
      jpeg_encode_image(img, img_jpeg, 99, TRUE);

      FILE *fp = fopen(save_name, "w");
      if (fp == NULL) {
        printf("[video_thread-thread] Could not write shot %s.\n", save_name);
      } else {
        // Save it to the file and close it
        fwrite(img_jpeg->buf, sizeof(uint8_t), img_jpeg->buf_size, fp);
        fclose(fp);
      }
//#endif

      // We don't need to seek for a next index anymore
      break;
    }
  }
}
