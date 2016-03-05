/*
 * Copyright (C) nikhil
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/testg4_module/orange_follow.c"
 * @author nikhil
 * Orange Following
 */

#include "modules/testg4_module/orange_follow.h"
//#include "modules/testg4_module/wpg4_module.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/computer_vision/colorfilter.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "state.h"
#include <stdlib.h>

typedef uint8_t yuv_fil_colour[6];

int thresholdColourCount = 150;
int maxColourCount = 400;
int thresholdHeading = 20; //Threshold for number of pixels from center for correct heading
uint16_t imageWidth = 272; //Hardcoded image size in video_thread.c

// Define the colours here

yuv_fil_colour c_orange={1,1,1,1,1,1};

// Functions here
void follow_colour_init(yuv_fil_colour* setColour){
	color_lum_min=*setColour[0];
	color_lum_max=*setColour[1];
	color_cb_min=*setColour[2];
	color_cb_max=*setColour[3];
	color_cr_min=*setColour[4];
	color_cr_max=*setColour[5];
}

// Check if heading is towards colour and adjust heading as necessary

void follow_check_periodic(struct point_t *points, uint16_t points_cnt, int32_t headingIncrement){
	int px_x_count=0;
	double px_x_avg = 0.0;
	//Check if count of points is above the threshold otherwise no action
	if(color_count > thresholdColourCount){

		//Add x axis component for all detected points
		for (int i = 0; i < points_cnt; i++){
			px_x_count += points[i].x;
		}

		//Calculate the average x pixel for all detected points
		px_x_avg = ((double)px_x_count)/points_cnt;
		px_x_avg = ((int)px_x_avg) - imageWidth/2;

		//Check if the px_x_avg is within thresholdHeading of centre of image
		if(abs(px_x_avg) < thresholdHeading){

			// If the avg pixel is on the right then we want UAV heading to go clockwise
			if(px_x_avg > 0){
				increase_nav_heading(&nav_heading,headingIncrement, TRUE);
			} else {
				increase_nav_heading(&nav_heading,(-1*headingIncrement), TRUE);
			}

		}

	}

}

// Check whether keepMoving is okay

uint8_t follow_keepmove(){
	//Check if colour_count within bounds and not too close
	if(color_count > thresholdColourCount && color_count <= maxColourCount){
		return TRUE;
	} else {
		return FALSE;
	}
}



