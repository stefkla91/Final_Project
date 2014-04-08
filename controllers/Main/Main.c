/**
 * File:          Main.c
 * Date:          15.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: V0.3
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

#include "../../lib/odometry.h"
#include "../../lib/e_puck_movement.h"
#include "../../lib/map_building.h"
#include "../../lib/e_puck_distance_sensors.h"

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
//robots initial positions on the map 
/* int robot_x = MAP_SIZE / 2;
int robot_y = MAP_SIZE / 2; 
 */
//odometry
struct odometryTrackStruct ot;
/**
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv){
	/* double dSpeed = 300.0f;
	double dDistance = 0.4f;  */
	
	//initialize and reset all needed devices 
	wb_robot_init();
	reset();
	
	odometry_track_start(&ot);
	
	ot.result.x = 0;// 0.008;
	ot.result.y = 0;//0.008;
	ot.result.theta = 4.71238898;// in RAD = 270 degrees 

	while (wb_robot_step(TIME_STEP*4) != -1) {
	/*	move_forward(speed, distance);
		turn_right(speed);
		turn_left(speed); */
		odometry_track_step(&ot);
	
		run(&ot);
		//UMBmark(dSpeed, dDistance);
		
		
		
		// update position on the map
		/*robot_x = wtom(ot.result.x);
		robot_y = wtom(ot.result.y); 
		wb_display_set_color(display,0xFF0000);
		wb_display_draw_rectangle(display,robot_x, display_height-robot_y-1,1,1);  */
	};
	wb_robot_cleanup();

	return 0;
}
