/**
 * File:          functions.c
 * Date:          11.04.2014
 * Description:   This file holds functions which are not feature specific for the 
 				  project. 
 * Author:        Stefan Klaus
 * Modifications: v1.0
 */

#include <stdio.h>

#ifndef M_PI
	#define M_PI 3.1415926535897932384626433832795L
#endif

#define RTOD(r) ((r) * 180 / M_PI)
#define ANGLE_TOLERANCE 20
#define EAST 0 
#define NORTH 90
#define WEST 180
#define SOUTH 270

//function declarations
 int check_direction(double d);
 int return_angle(double rad);

/**
Returns an int which represents the direction the robot is travelling in.
The directions are specified which the start view of the simulator in mind 
meaning up = north, to the right = east, down = south and to the left = west.
These directions are represented as follows:

1 = north
2 = east
3 = south
4 = west
*/
int check_direction(double d){
	int i = return_angle(d);
	int result;

	if(i + ANGLE_TOLERANCE >= 360){
		i -= 360; 
	} 
	
	if(NORTH < i + ANGLE_TOLERANCE && NORTH > i - ANGLE_TOLERANCE){
		result = 1;
	}else if(EAST < i + ANGLE_TOLERANCE && EAST > i - ANGLE_TOLERANCE){
		result = 2;
	}else if(SOUTH < i + ANGLE_TOLERANCE && SOUTH > i - ANGLE_TOLERANCE){
		result = 3;
	}else if(WEST < i + ANGLE_TOLERANCE && WEST > i - ANGLE_TOLERANCE){
		result = 4; 
	}
	return result;
}

/**
returns the angle in which the robot is moving
*/
int return_angle(double rad){
	double rotation;
	if(RTOD(rad) < 0){
		rotation = RTOD(rad) + 360;
	}else{
		rotation = RTOD(rad); 
	}

	printf("%f\n", rotation);
	return rotation;
}