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

#include "modules/testmodule/testmodule.h"

extern void spentLongTimeInBlock (int timeInBlock){
	return timeInBlock > 5;
}

