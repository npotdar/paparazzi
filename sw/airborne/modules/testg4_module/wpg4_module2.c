/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/wpg4_module/wpg4_module.c"
 * @author nikhil
 * Moving waypoints relative
 */

#include "modules/wpg4_module/wpg4_module.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <stdlib.h>

uint8_t increase_nav_heading(int32_t *heading, int32_t increment, uint8_t yaw_enable)
{
  struct Int32Eulers new_state;
  struct Int32Eulers *cur_state=stateGetNedToBodyEulers_i();

  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  // Yaw also if TRUE
  if(yaw_enable){
	  new_state=*cur_state;
	  new_state.psi = RadOfDeg(*heading);
	  stateSetNedToBodyEulers_i(&new_state);
  }

  return FALSE;
}

uint8_t MoveRelative(uint8_t waypoint, float distanceMeters){
	struct EnuCoor_i new_cord;
	struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get current position for craft in ENU Coordinates

	// Calculate the sine and cosine of the heading the drone is keeping
	float sin_heading = sinf(RadOfDeg(nav_heading));
	float cos_heading = cosf(RadOfDeg(nav_heading));

	// Now determine where to place the waypoint you want to go to
	new_cord.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	new_cord.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	new_cord.z = pos->z; // Keep the height the same
	printf("%d : Sin heading %f, Cos heading %f, New X %f, New Y %f \n",nav_heading, sin_heading,cos_heading,new_cord.x,new_cord.y);
	// Set the waypoint to the calculated position
	waypoint_set_xy_i(waypoint, new_cord.x, new_cord.y);

	return FALSE;
}
