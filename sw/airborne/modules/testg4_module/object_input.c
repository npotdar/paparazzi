#include <stdio.h>
#include <math.h>
#include "object_input.h"


#define n 10
int fov = 3;
float threshold = 10.0;
float TURN = 0;
//int TURN_R = 0;
//int TURN_B = 0;
int update = 1;

double depth[n] =  {6.2, 42.5, 33.4, 52.2, 55.0, 5.2, 56.2, 8.5, 55.3, 2.1};

float objTurnCommand(void)
{
	if (update==1){
		int i;
		for (i = n/2 -1 - fov ; i < n/2 + fov; i++){
			if (depth[i] < threshold && TURN == 0){
				if (i < n/2) {
				TURN = 15;  //+1 turn right
				}
				else if (i >= n/2){
				TURN = -15; //-1 turn left
				}
			}
		}
		update = 1;
	}
	else{
		TURN = 0;
	}
	return TURN;
}
