/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/testg4_module/brg4_module.h"
 * @author nikhil
 * Moving waypoints relative
 */

#ifndef BRMOVE_H
#define BRMOVE_H
#include <inttypes.h>

//typedef uint8_t yuv_fil_colour[6];

extern uint8_t detectedColour;
extern int detectedRed;
extern int detectedBlue;
extern int switchColour;

extern void br_colour_init(int setColour[6]);
extern void br_colour_periodic(int setColour1[6], int setColour2[6]);
extern void br_detected_periodic(void);

extern int c_see_red[6];
extern int c_see_blue[6];

#endif

