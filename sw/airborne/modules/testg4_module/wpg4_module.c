/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/testg4_module/wpg4_module.c"
 * @author nikhil
 * Moving waypoints relative
 */

#include "modules/testg4_module/testg4_module.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <stdlib.h>

uint8_t HeadMoveRelative(int32_t *heading, int32_t increaseDeg, uint8_t waypoint, float distanceMeters){
	struct EnuCoor_i new_cord;
	struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get current position for craft in ENU Coordinates

	// Increase the heading of the craft by the increaseDeg in degrees to initiate movement in direction
	*heading = *heading + increaseDeg;
	INT32_ANGLE_NORMALIZE(*heading);

	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	// Now determine where to place the waypoint you want to go to
	new_cord.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	new_cord.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	new_cord.z = pos->z; // Keep the height the same

	// Set the waypoint to the calculated position
	waypoint_set_xy_i(waypoint, new_cord.x, new_cord.y);

	return FALSE;
}
