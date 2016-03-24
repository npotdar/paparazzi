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
int OrangeDet = 1;
int OrangePos = 120;
int turn_angle = 45;


double depth[n] =  {6.2, 42.5, 33.4, 13.2, 55.0, 5.2, 56.2, 8.5, 55.3, 2.1};

float main(void)
{
	if (update==1){
		int i;

		/* CHECK ORANGE */
		if (OrangeDet == 1){
			if (OrangePos < 150){
				TURN = turn_angle;
				printf("detect orange on the left, turn: %f degrees", TURN);
			}
			else if(OrangePos >= 150){
				TURN = -1* turn_angle;
				printf("detect orange on the right, turn: %f degrees", TURN);
			}
		}
		else{
			TURN = 0;
		}
		/* CHECK OBJECT */
		for (i = n/2 -1 - fov ; i < n/2 + fov; i++){
			if (depth[i] < threshold && TURN == 0){
				if (i < n/2) {
				TURN = turn_angle;  //positive angle turn right
				printf("detect object on the left, turn: %f degrees", TURN);
				}
				else if (i >= n/2){
				TURN = -1*turn_angle; // negative angle turn left
				printf("detect object on the right, turn: %f degrees", TURN);
				}
			}
		}
		update = 0;
	}
	/* IF NOTHING DETECTED */
	else{
		TURN = 0;
	}
	return TURN;
}
