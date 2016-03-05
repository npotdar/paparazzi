/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/testg4_module/brg4_module.c"
 * @author nikhil
 * Moving waypoints relative
 */

#include "modules/testg4_module/brg4_module.h"
//#include "modules/testg4_module/wpg4_module.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/computer_vision/colorfilter.h"
#include "state.h"
#include <stdlib.h>

typedef uint8_t yuv_fil_colour[6];

uint8_t detectedColour = FALSE;
int detectedRed = FALSE;
int detectedBlue = FALSE;
int thresholdColourCount = 150;
int switchColour = 1;

// Define the colours here

yuv_fil_colour c_red={1,1,1,1,1,1};
yuv_fil_colour c_blue={1,1,1,1,1,1};


void br_colour_init(yuv_fil_colour* setColour){
	color_lum_min=*setColour[0];
	color_lum_max=*setColour[1];
	color_cb_min=*setColour[2];
	color_cb_max=*setColour[3];
	color_cr_min=*setColour[4];
	color_cr_max=*setColour[5];
}

void br_colour_periodic(yuv_fil_colour* setColour1, yuv_fil_colour* setColour2){
	if(switchColour==0){
		color_lum_min=*setColour1[0];
		color_lum_max=*setColour1[1];
		color_cb_min=*setColour1[2];
		color_cb_max=*setColour1[3];
		color_cr_min=*setColour1[4];
		color_cr_max=*setColour1[5];
		switchColour = 1;
	} else {
		color_lum_min=*setColour2[0];
		color_lum_max=*setColour2[1];
		color_cb_min=*setColour2[2];
		color_cb_max=*setColour2[3];
		color_cr_min=*setColour2[4];
		color_cr_max=*setColour2[5];
		switchColour = 0;
	}
}

void br_detected_periodic(){
	detectedColour = color_count > thresholdColourCount;
	printf("Check colour detect: %d, colour: %d \n",color_count,switchColour);
	if(detectedColour && switchColour==1){
		detectedRed = TRUE;
	} else if(detectedColour && switchColour==0){
		detectedBlue = TRUE;
	}
}


