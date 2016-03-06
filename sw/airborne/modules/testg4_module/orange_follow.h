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

#ifndef ORANGE_FOLLOW_H
#define ORANGE_FOLLOW_H
#include <inttypes.h>

typedef uint8_t yuv_fil_colour[6];

extern int thresholdColourCount;
extern int maxColourCount;
extern int thresholdHeading;
extern uint16_t headingIncrement;

extern void follow_colour_init(yuv_fil_colour* setColour, struct image_t *img);
extern void follow_check_periodic(int32_t headingIncrement);
extern uint8_t follow_keepmove();

#endif

