/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/testg4_module/wpg4_module.h"
 * @author nikhil
 * Moving waypoints relative
 */

#ifndef WPMOVE_H
#define WPMOVE_H
#include <inttypes.h>

extern uint8_t HeadMoveRelative(int32_t *heading, int32_t increaseDeg, uint8_t waypoint, float distanceMeters);

#endif

