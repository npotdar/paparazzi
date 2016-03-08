/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/testg4_module/testg4_module.c"
 * @author nikhil
 * test for crash course
 */

#include "modules/testg4_module/testg4_module.h"
#include "firmwares/rotorcraft/navigation.h"
#include <stdlib.h>

extern uint8_t spentLongTimeInBlock (int timeInBlock){
	printf("Spent too long in block \n");
	return timeInBlock > 5;
}

extern uint8_t testWpId(uint8_t waypoint){
	printf("waypoint x coordinate is");
	return true;
}

