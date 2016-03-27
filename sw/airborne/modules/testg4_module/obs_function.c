/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @author nikhil
 */

#include "modules/testg4_module/obs_function.h"
//#include "modules/testg4_module/wpg4_module.h"
#include <state.h>
#include <stdlib.h>

uint8_t RANDOM = 0;
uint8_t COUNTDOWN = 0;
uint8_t OBS_DETECT = FALSE;

float obs_turn() {
	OBS_DETECT = FALSE;
	float headArray[10] = { 45, 45, 90, -90, -45, 45, 90, 90, -90, 45 };
	RANDOM++;
	if (RANDOM == 10) {
		RANDOM = 0;
	}
	printf("TURN: %f \n", headArray[RANDOM]);
	return headArray[RANDOM];
}

void obs_periodic() {
	if (COUNTDOWN%10== 1) {
		OBS_DETECT = TRUE;
	}
	COUNTDOWN++;
	printf("OBS_DETECT: %d, %d \n", OBS_DETECT, COUNTDOWN);
}
