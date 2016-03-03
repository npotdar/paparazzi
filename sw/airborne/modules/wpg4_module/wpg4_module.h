/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/wpg4_module/wpg4_module.h"
 * @author nikhil
 * Moving waypoints relative
 */

#ifndef WPMOVE_H
#define WPMOVE_H
#include <inttypes.h>

extern uint8_t MoveRelative(uint8_t waypoint, float distanceMeters);
extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment, uint8_t yaw_enable);

#endif

