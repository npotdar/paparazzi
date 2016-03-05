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

uint8_t detectedColour;
extern int detectedRed;
extern int detectedBlue;

extern void br_colour_init(yuv_fil_colour* setColour,int32_t *heading);
extern void br_colour_periodic(yuv_fil_colour* setColour1, yuv_fil_colour* setColour2);
extern void br_detected_periodic();

typedef uint8_t yuv_fil_colour[6];

extern yuv_fil_colour c_red;
extern yuv_fil_colour c_blue;

#endif

