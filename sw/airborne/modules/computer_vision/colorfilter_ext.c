/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.c
 *
 * Extended color filter that also outputs the average x and y position of detected points
 */

// Own header
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/colorfilter_ext.h"
#include "modules/computer_vision/lib/vision/image.h"

#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <std.h>

// Filter Settings
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

// Result

int color_count = 0;
int color_avg_x = 0;
int color_avg_y = 0;

// Function
bool_t colorfilter_ext_func(struct image_t* img);
bool_t colorfilter_ext_func(struct image_t* img)
{

	// Filter
  image_yuv422_colorfilt_ext(img,img,&color_count,&color_avg_x,&color_avg_y,
        color_lum_min,color_lum_max,
        color_cb_min,color_cb_max,
        color_cr_min,color_cr_max
        );

  DOWNLINK_SEND_COLORFILTER(DefaultChannel, DefaultDevice, &color_count);
  return FALSE;
}

void colorfilter_ext_init(void)
{
  cv_add(colorfilter_ext_func);
}

