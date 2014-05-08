/**
 * File:          e_puck_movement.c
 * Date:          10.03.2014
 * Description:   Holds all major movement algorithms, as well as the callibration algorithm
 *				  UMBmark
 * Author:        Stefan Klaus
 * Modifications: V 1.0
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
// #include <webots/light_sensor.h>
// #include <webots/camera.h>
// #include <webots/accelerometer.h>
#include <webots/led.h>
#include <math.h>
#include "e_puck_movement.h"
#include "map_building.h"
#include "odometry.h"

#ifndef M_PI
	#define M_PI 3.1415926535897932384626433832795L
#endif

#define TIME_STEP 8 
#define WHEEL_RADIUS 0.0206625 // according to webots
#define LEFT_DIAMETER 0.0416//orig 0.0416
#define RIGHT_DIAMETER 0.0404//orig 0.0404
#define WHEEL_DIAMETER (LEFT_DIAMETER + RIGHT_DIAMETER)
#define WHEELBASE 0.058 //orig 0.052, updated version 0.058
#define ENCODER_RESOLUTION 159.23 //orig 159.23
#define INCREMENT_STEP 1000 //steps of the motor for a whole rotation orig REV_STEP
#define STEP_TOLERANCE 6.0
#define RANGE (1024 / 2)
#define MIN_DIST 20.0f //minimum distance before slowing down, 20,0 original
#define MIN_SPEED 10.0f //speed when slowing down
#define NUMTOURNAMENTS 1 //5
#define RTOD(r) ((r) * 180 / M_PI)
#define ANGLE_TOLERANCE 5
#define EAST 0 
#define NORTH 90
#define WEST 180
#define SOUTH 270

double dCurSpeed[2] = {0.0f, 0.0f}; // global buffer to store the current speed values. orig:dMSpeed
double dPrevEncPos[2] = {0.0f, 0.0f}; // global buffer to save the previous encoder postions.
double *point_dOdometryData; // global buffer to save the current odometry data.
WbDeviceTag led[3]; // LEDs.

// motion control methods:
void stop_robot();
void move_forward(double dSped, double dDis, struct odometryTrackStruct * ot); //, struct odometryTrackStruct * ot
void turn_left(double dSpeed);
void turn_right(double dSpeed);
void turn_angle(double dAngle, double dSpeed); 
void set_motor_speed(double dSpeedL, double dSpeedR);
double* get_encoder_positions();
void check_rotation(double cur_rot, double want_rot, double dSpeed);

// odometry:
double* compute_odometry_data();

// odometry UMBmark:
void UMBmark(double dSpeed, double dDistance, struct odometryTrackStruct * ot); //calibration 
void measure_clockWise(double dSpeed, double dDistance, struct odometryTrackStruct * ot);
void measure_CounterClockWise(double dSpeed, double dDistance, struct odometryTrackStruct * ot); 

// braitenberg weights for wall following
float weights_left[8] = {-1,-1,-1,0.5,-0.5,0.5,1,2};
float weights_right[8] = {1,0.8,1,-0.5,0.5,-1,-1.6,-2};

// LEDs:
void set_leds(int iActive);

 /**
 Function to stop the robot
 */
void stop_robot() {
	dCurSpeed[0] = 0.0f;
	dCurSpeed[1] = 0.0f;
	wb_differential_wheels_set_speed(dCurSpeed[0], dCurSpeed[1]);
}

/**
Function to move the robot forward a given distance at a given speed
*/
void move_forward(double dSpeed, double dDist, struct odometryTrackStruct * ot){ //, struct odometryTrackStruct * ot
	double dStepCount = 0.0f;
	double dStopPosLeft = 0.0f;
	double dStopPosRight = 0.0f;
	double *point_dEncPos;	
	
	if((dDist > 0.0f) && (dSpeed > 0.0f)){
		//calculate the number of steps
		dStepCount = (INCREMENT_STEP/(M_PI * WHEEL_DIAMETER / 2)) * dDist;
		
		/*read the current encoder positions of both wheels and
		calculate the encoder positions when the robot has to
		stop at the given distance ... */
		point_dEncPos = get_encoder_positions();
		
		dStopPosLeft = point_dEncPos[0] + dStepCount;
		dStopPosRight = point_dEncPos[1] + dStepCount;
		
		//compute odometry data
		point_dOdometryData = compute_odometry_data();
		odometry_track_step(ot);
		
		
		//set speed
		set_motor_speed(dSpeed, dSpeed); 
		
		//step tolereance test
		while((point_dEncPos[0] < dStopPosLeft) && (point_dEncPos[1] < dStopPosRight)){
			//get odometry data
			point_dOdometryData = compute_odometry_data();
			odometry_track_step(ot);
			//get wheel encoders
			point_dEncPos = get_encoder_positions();
			
			//slow down the closer the robot come to the destination, reduces the error
			if(fabs(dStopPosLeft - point_dEncPos[0]) <= MIN_DIST){
				set_motor_speed(MIN_SPEED, MIN_SPEED);
			}
		}
	}
	stop_robot();
	
	//update odometry data
	point_dOdometryData = compute_odometry_data();
	odometry_track_step(ot);
	wb_robot_step(TIME_STEP);
}

/**
Function to turn left
*/
void turn_left(double dSpeed){
	/*turning 100 degrees actually turn the robot ~90 degrees.
	most likely because the wheel radius I have found does not match the 
	one in the simulator*/
	turn_angle(-90.0f, dSpeed); 
}
/**
Function to turn right
*/
void turn_right(double dSpeed){
	/*turning 100 degrees actually turn the robot ~90 degrees.
	most likely because the wheel radius I have found does not match the 
	one in the simulator*/
	turn_angle(90.0f, dSpeed);
}

/**
Function to turn the robot a given angle with a given speed
*/
void turn_angle(double dAngle, double dSpeed){
	double dFactor = 0.0f;
	double dStepCount = 0.0f;
	double dStopPosLeft = 0.0f;
	double dStopPosRight = 0.0f;
	double *point_dEncPos;
	
	if((dAngle != 0.0f) && (dSpeed > 0.0f)){
		//calculate turn factor
		dFactor = fabs(360.0f/dAngle);
		
		//calculate the number of step counts for the rotations
		dStepCount = (INCREMENT_STEP * WHEELBASE)/(dFactor * WHEEL_DIAMETER / 2);
		
		point_dEncPos = get_encoder_positions();
		
		//turn right
		if(dAngle > 0){
			//calculate the target encoder positions
			dStopPosLeft = point_dEncPos[0] + dStepCount;
			dStopPosRight = point_dEncPos[1] - dStepCount;
			
			point_dOdometryData = compute_odometry_data();
			
			set_motor_speed(dSpeed, -dSpeed);
			
			while((point_dEncPos[0] < dStopPosLeft) && (point_dEncPos[1] > dStopPosRight)){
				//get odometry data
				point_dOdometryData = compute_odometry_data();
				
				//get wheel encoders
				point_dEncPos = get_encoder_positions();
				
				//slow down the closer the robot come to the destination, reduces the error
				if(fabs(dStopPosLeft - point_dEncPos[0]) <= MIN_DIST){
					set_motor_speed(MIN_SPEED, -MIN_SPEED);
				}
			}	
		
	} else { // turn left ...
		dStopPosLeft = point_dEncPos[0] - dStepCount;
		dStopPosRight = point_dEncPos[1] + dStepCount;
		
		point_dOdometryData = compute_odometry_data();

		// turn left the robot ...
		set_motor_speed(-dSpeed, dSpeed);

		while((point_dEncPos[0] > dStopPosLeft) &&(point_dEncPos[1] < dStopPosRight)){

			point_dOdometryData = compute_odometry_data();
			point_dEncPos = get_encoder_positions();
			if( fabs(dStopPosLeft - point_dEncPos[0]) <= MIN_DIST ){
				set_motor_speed(-MIN_SPEED, MIN_SPEED); }
			}
		}
	}
	stop_robot();
	
	//update odometry data
	point_dOdometryData = compute_odometry_data();
	wb_robot_step(TIME_STEP);
}

/**
Function to set the motor speed of the robot
*/
void set_motor_speed(double dLeftSpeed, double dRightSpeed){
	dCurSpeed[0] = dLeftSpeed;
	dCurSpeed[1] = dRightSpeed;
	
	wb_differential_wheels_set_speed(dCurSpeed[0], dCurSpeed[1]);
}

/**
Function to get and return the current encoder position of the wheels
*/
double* get_encoder_positions(){
	static double dEncPos[2];
	
	//read encoder positions
	dEncPos[0] = wb_differential_wheels_get_left_encoder();
	dEncPos[1] = wb_differential_wheels_get_right_encoder();
	
	wb_robot_step(TIME_STEP);
	
	return dEncPos;
}

/**
NOTE:
This function has no practical use as the odometry data it generates is not used in anyway
However removing it led to errors in the movement and turning algorithms and it was not enough
time to fix these errors.


Function to compute the robots current odometry data
this includes the x and y placement as well as the rotation theta
*/
double* compute_odometry_data(){
	static double dOdometryData[3];
	double *point_dEncPos;
	
	double dNumTicksLeft = 0.0f;
	double dNumTicksRight = 0.0f;
	double dLeftDist = 0.0f;
	double dRightDist = 0.0f;
	double dDispCenter = 0.0f;
	static double dTheta = 0.0f;
	static double dPosX = 0.0f;
	static double dPosY = 0.0f;
	
	//get encoder position
	point_dEncPos = get_encoder_positions();
	
	/* calculate the distance travled for the wheels
	(number of ticks done since the previous encoder position
	was recoreded)*/
	dNumTicksLeft = point_dEncPos[0] - dPrevEncPos[0];
	dNumTicksRight = point_dEncPos[1] - dPrevEncPos[1];
	
	//convert the number of encoder ticks to a real distance
	dLeftDist = dNumTicksLeft / ENCODER_RESOLUTION * WHEEL_RADIUS;
	dRightDist = dNumTicksRight / ENCODER_RESOLUTION * WHEEL_RADIUS;
	
	//calculate the displacement distance from the center 
	dDispCenter = (dLeftDist + dRightDist)/2.0f;
	
	//calculate the rotation theta of the robot
	dTheta += (dLeftDist - dRightDist)/(WHEELBASE);
	
	//calculate Cartesian coordinates (x, y) of the robot
	dPosX += dDispCenter * sin(dTheta);
	dPosY += dDispCenter * cos(dTheta); 
	
	//update the previous encoder positions
	dPrevEncPos[0] = point_dEncPos[0];
	dPrevEncPos[1] = point_dEncPos[1];
	
	//write the calculate values into the buffer and return
	dOdometryData[0] = dPosX;
	dOdometryData[1] = dPosY;
	dOdometryData[2] = dTheta;
	
	return dOdometryData;
}
/**
University of Michigan Benchmark
*/
void UMBmark(double dSpeed, double dDistance, struct odometryTrackStruct * ot){	
	//move the robot clockwise and counterclockwise 
	measure_clockWise(dSpeed, dDistance, ot);
	
	measure_CounterClockWise(dSpeed, dDistance, ot);
	
	stop_robot();
} 
/**
Function to measure the movement accuracy by driving
a clockwise square.
This is part of the UMBmark algorithm
*/
void measure_clockWise(double dSpeed, double dDistance, struct odometryTrackStruct * ot){
	int i, j;
	
	for(i = 0;i < NUMTOURNAMENTS; i++){
		for(j = 0;j < 4;j++){
			move_forward(dSpeed, dDistance, ot);
			turn_right(dSpeed);
		}
		wb_robot_step(TIME_STEP);
	}
}  

/**
Function to measure the movement accuracy by driving
a counter-clockwise square.
This is part of the UMBmark algorithm
*/
void measure_CounterClockWise(double dSpeed, double dDistance, struct odometryTrackStruct * ot){
	int i, j; 
	
	//turn the robot right for moving the same square counter clock wise
	turn_right(dSpeed);
	
	for(i = 0;i < NUMTOURNAMENTS;i++){		
		for(j = 0;j < 4; j++){
			move_forward(dSpeed, dDistance, ot);
			turn_left(dSpeed);
		}
		wb_robot_step(TIME_STEP);
	}
} 

/**
set the status of the LEDs
*/
void set_leds(int iActive){
	int i;
	for(i = 0; i < 3;i++){
		wb_led_set(led[1], iActive);
	}
}

/**
Function to compare the current heading to the wanted heading
and fix the heading should it surpass a threshold
*/
void check_rotation(double cur_rot, double want_rot, double dSpeed){
	double dthreshold = 1; //2,0
	double diff;
	char text[] = "Correcting";
	if(cur_rot + 20 >= 360){
		cur_rot -= 360; 
	} 
	
	if(cur_rot > want_rot + dthreshold){
		diff = cur_rot - want_rot;
		printf("%s\n", text);
		turn_angle(diff, dSpeed);
	}else if(cur_rot < want_rot - dthreshold){
		diff = want_rot - cur_rot;
		printf("%s\n", text);
		turn_angle(-diff, dSpeed);
	}
}
