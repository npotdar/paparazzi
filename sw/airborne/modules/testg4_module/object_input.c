#include <stdio.h>
#include <math.h>
#include "object_input.h"


#define n 5
int fov = 2;
float threshold = 10.0;
float TURN = 0;
//int TURN_R = 0;
//int TURN_B = 0;
int update = 1;
int OrangeDet = 1;
int OrangePos = 120;
int turn_angle = 45;


double depth[n] =  {0.003, 0.003, 3.0424, 4.3203, inf};

float main(void)
{
	if (update==1){
		int i;

		/* CHECK ORANGE */
		if (OrangeDet == 1){
			if (OrangePos < 136){
				TURN = turn_angle;
				printf("detect orange on the left, turn: %f degrees", TURN);
			}
			else if(OrangePos >= 136){
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
