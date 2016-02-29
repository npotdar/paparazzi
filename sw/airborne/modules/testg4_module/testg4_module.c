/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/testmodule/testmodule.c"
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

