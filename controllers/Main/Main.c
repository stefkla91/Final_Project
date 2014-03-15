/*
 * File:          Main.c
 * Date:          15.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: v0.1
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/accelerometer.h>
#include <webots/led.h>
#include <math.h>

#include "../../lib/e_puck_movement.h"
#include "../../lib/odometry.h"
#include "../../lib/map_building.h"

#define TIME_STEP 8
#define MAP_SIZE 70

//occupancy code:
#define THRESHOLD_DIST 100
#define OCCUPANCE_DIST 150
#define LEFT 0        // Left side
#define RIGHT 1       // right side
// 8 IR proximity sensors
#define NUM_DIST_SENS 8
#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7

/* Definitions */
//Leds
WbDeviceTag led[3];

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv){
	/* double speed = 300.0f;
	double distance = 0.05f; */
	
	//initialize and reset all needed devices 
	wb_robot_init();
	reset();

	while (wb_robot_step(TIME_STEP*4) != -1) {
		//move_forward(speed, distance);
		run();
	};
	wb_robot_cleanup();

	return 0;
}
