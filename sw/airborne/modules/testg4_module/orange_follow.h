/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/testg4_module/orange_follow.h"
 * @author nikhil
 * Orange following
 */

#ifndef BRMOVE_H
#define BRMOVE_H
#include <inttypes.h>

typedef uint8_t yuv_fil_colour[6];

extern uint8_t detectedColour;

extern yuv_fil_colour c_red;
extern yuv_fil_colour c_blue;

extern void follow_colour_init(yuv_fil_colour* setColour);

#endif

