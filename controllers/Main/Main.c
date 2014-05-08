/**
 * File:          Main.c
 * Date:          15.03.2014
 * Description:   Main file of the project
 * Author:        Stefan Klaus
 * Modifications: v1.0
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
// #include <webots/differential_wheels.h>
// #include <webots/distance_sensor.h>
// #include <webots/light_sensor.h>
// #include <webots/camera.h>
// #include <webots/display.h>
// #include <webots/accelerometer.h>
// #include <webots/led.h>
#include <math.h>

#include "../../lib/functions.h"
#include "../../lib/odometry.h"
#include "../../lib/reference_points.h"
#include "../../lib/e_puck_movement.h"
#include "../../lib/map_building.h"
#include "../../lib/e_puck_distance_sensors.h"


#define TIME_STEP 8
#define MAP_SIZE 70


/* Definitions */
//Leds
WbDeviceTag led[3];

//odometry
struct odometryTrackStruct ot;

//references
struct referencePos ref;
/**
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv){
	// double dSpeed = 300.0f; //only used for UMBmark
	// double dDistance = 0.3f;  //used ofr UMBmark
	
	//initialize and reset all needed devices 
	wb_robot_init();
	reset();
	
	odometry_track_start(&ot); //initialize the odometry struct

	// ref.lower_left.x = 0;
	// ref.lower_left.y = 0;
	// ref.lower_right.x = 0;
	// ref.lower_right.y = 0;
	// ref.upper_left.x = 0;
	// ref.upper_left.y = 0;
	// ref.upper_right.x = 0;
	// ref.upper_left.y = 0;

	ot.result.x =0;// 0.199997;// 0.00;
	ot.result.y = 0;//0.00133553;//0.008;
	ot.result.theta = 4.71238898;//4.71238898 in RAD = 270 degrees 

	while (wb_robot_step(TIME_STEP*4) != -1) {
		odometry_track_step(&ot);
	
		run(&ot, &ref);
	//	UMBmark(dSpeed, dDistance);
	};
	wb_robot_cleanup();

	return 0;
}
