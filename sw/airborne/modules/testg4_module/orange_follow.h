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

extern int thresholdColourCount;
extern int maxColourCount;
extern int thresholdHeading;
extern uint8_t headingIncrement;

extern int c_see_orange[6];

extern void follow_colour_init(int setColour[6]);
extern void follow_check_periodic(void);
extern uint8_t follow_keepmove(void);

#endif

