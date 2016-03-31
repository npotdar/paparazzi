#ifndef MAVG4_OPTICFLOW_H
#define MAVG4_OPTICFLOW_H

#include "std.h"
#include "lib/vision/image.h"
#include "lib/v4l/v4l2.h"

struct opticflow_t {
  bool_t got_first_img;             ///< If we got a image to work with

  struct image_t img_gray;          ///< Current gray image frame
  struct image_t prev_img_gray;     ///< Previous gray image frame
  struct timeval prev_timestamp;    ///< Timestamp of the previous frame, used for FPS calculation

  uint8_t max_track_corners;        ///< Maximum amount of corners Lucas Kanade should track
  uint16_t window_size;             ///< Window size of the Lucas Kanade calculation (needs to be even)
  uint8_t subpixel_factor;          ///< The amount of subpixels per pixel
  uint8_t max_iterations;           ///< The maximum amount of iterations the Lucas Kanade algorithm should do
  uint8_t threshold_vec;            ///< The threshold in x, y subpixels which the algorithm should stop

  bool_t fast9_adaptive;            ///< Whether the FAST9 threshold should be adaptive
  uint8_t fast9_threshold;          ///< FAST9 corner detection threshold
  uint16_t fast9_min_distance;      ///< Minimum distance in pixels between corners
};

/* The result calculated from the opticflow */
struct opticflow_result_t {
  float fps;              ///< Frames per second of the optical flow calculation
  uint16_t corner_cnt;    ///< The amount of coners found by FAST9
  uint16_t tracked_cnt;   ///< The amount of tracked corners
  uint8_t obs_detect;	  ///< Obstacle detection
  float obs_heading;	  ///< Obstacle heading change
  float flows[5];		  ///< Detection flows depth segmented
};

// Required for settings
extern struct opticflow_t opticflow;

// Require for obstacle detection
extern uint8_t TRANS_MOVE;

extern float DETECT_THRESHOLD;
extern float OBS_HEADING_SET;

extern uint8_t OBS_DETECT;
extern float OBS_DETECT_S;
extern float OBS_HEADING;

extern float IMGERROR_THRESHOLD;

// Module functions
extern void opticflow_module_init(void);
extern void opticflow_module_run(void);
extern void opticflow_module_start(void);
extern void opticflow_module_stop(void);
void opticflow_calc_frame(struct opticflow_t *opticflow, struct image_t *img, struct opticflow_result_t *result);
extern float obs_heading(void);
extern uint8_t trans_false(void);
extern uint8_t trans_true(void);

#endif
