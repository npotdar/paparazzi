/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @author nikhil
 */

 #ifndef OBS_H
#define OBS_H

#include <inttypes.h>

 extern uint8_t OBS_DETECT;
 extern float obs_turn(void);
 extern void obs_periodic(void);

#endif
