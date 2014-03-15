/*
 * File:          map_building.c
 * Date:          15.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: v0.1
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <math.h>
#include <webots/display.h> 

#include "map_building.h"
#include "e_puck_movement.h"
#include "odometry.h"

#define TIME_STEP 8
#define MAP_SIZE 70
#define CELL_SIZE 0.015

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

//Distance sensors and corresponding arrays
WbDeviceTag ps[NUM_DIST_SENS];

//display and map 
WbDeviceTag display;
WbImageRef background;
int display_width;
int display_height;
int map[MAP_SIZE][MAP_SIZE];

//robots initial positions on the map 
int robot_x = MAP_SIZE / 2;
int robot_y = MAP_SIZE / 2; 

//Distance sensors and corresponding arrays
WbDeviceTag ps[NUM_DIST_SENS];
int ps_value[NUM_DIST_SENS]={0,0,0,0,0,0,0,0};
int obstacle[NUM_DIST_SENS]={0,0,0,0,0,0,0,0};
bool ob_front, ob_right, ob_left;

// this is the angle at which the IR sensors are placed on the robot
float angle_offset[NUM_DIST_SENS] = {0.2793, 0.7854, 1.5708, 2.618, -2.618, -1.5708, -0.7854, -0.2793};

//movement
int new_encoder;

//odometry struct
struct odometryTrackStruct ot;


/**
 * Initiate the display with a white color
 */
void init_display(){
	display = wb_robot_get_device("display");
	display_width = wb_display_get_width(display);
	display_height = wb_display_get_height(display);
	wb_display_fill_rectangle(display,0,0,display_width,display_height);
	background = wb_display_image_copy(display,0,0,display_width,display_height);
}

/**
 * Those 2 functions do the coordinate transform between the world coordinates (w)
 * and the map coordinates (m) in X and Y direction.
 */
static int wtom(float x)
{
	return (int)(MAP_SIZE / 2 + x / CELL_SIZE);
}

/**
 * Set the coresponding cell to 1 (occuM_PIed)
 * and display it
 */
void occupied_cell(int x, int y, float theta){

	// normalize angle
	while (theta > M_PI) {
		theta -= 2*M_PI;
	}
	while (theta < -M_PI) {
		theta += 2*M_PI;
	}

	// front cell
	if (-M_PI/6 <= theta && theta <= M_PI/6) {
		if (y+1 < MAP_SIZE) {
			map[x][y+1] = 1;
			wb_display_draw_rectangle(display,x,display_height-y,1,1);
		}
	}
	// right cell
	if (M_PI/3 <= theta && theta <= 2*M_PI/3) {
		if (x+1 < MAP_SIZE) {
			map[x+1][y] = 1;
			wb_display_draw_rectangle(display,x+1,display_height-1-y,1,1);
		}
	}
	// left cell
	if (-2*M_PI/3 <= theta && theta <= -M_PI/3) {
		if (x-1 > 0) {
			map[x-1][y] = 1;
			wb_display_draw_rectangle(display,x-1,display_height-1-y,1,1);
		}
	}
	// back cell
	if (5*M_PI/6 <= theta || theta <= -5*M_PI/6) {
		if (y-1 > 0) {
			map[x][y-1] = 1;
			wb_display_draw_rectangle(display,x,display_height-y-2,1,1);
		}
	}
}

/**
enables the needed sensor devices 
*/
void reset(){
	int it, i, j;

	//get the distance sensors
	char textPS[] = "ps0";
	for (it = 0;it < NUM_DIST_SENS;it++){
		ps[it] = wb_robot_get_device(textPS);
		textPS[2]++;
	}
	init_display();

	//enable the distance sensor and light sensor devices
	for(i = 0;i < NUM_DIST_SENS;i++){
		wb_distance_sensor_enable(ps[i], TIME_STEP);
	}

	/* //enable encoders
	wb_differential_wheels_enable_encoders(TIME_STEP);
	wb_differential_wheels_set_encoders(0,0); */
	
	// map init to 0
	for (i = 0; i < MAP_SIZE; i++) {
		for (j = 0; j < MAP_SIZE; j++) {
			map[i][j] = 0;
		}
	}
	
	//emable and initialize the wheel encoders
	wb_differential_wheels_enable_encoders(TIME_STEP*4);
	wb_differential_wheels_set_encoders(0.0f, 0.0f);
	
	// initialize the LEDs ...
	led[0] = wb_robot_get_device("led0");
	led[1] = wb_robot_get_device("led2");
	led[2] = wb_robot_get_device("led6");
}

void run(){
	int i;
	int robot_x = MAP_SIZE / 2;
	int robot_y = MAP_SIZE / 2;
	
	double dSpeed = 200.0f;
	double dDistance = 0.05f;
	
	// obstacle will contain a boolean information about a collision
	for(i=0;i<NUM_DIST_SENS;i++){
		ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
		obstacle[i] = ps_value[i] > THRESHOLD_DIST;
	} 
	
	//define boolean for sensor states for cleaner implementation
	bool ob_front = 
	obstacle[PS_RIGHT_10] ||
	obstacle[PS_LEFT_10];

	bool ob_right = 
	obstacle[PS_RIGHT_90];

	bool ob_left = 
	obstacle[PS_LEFT_90];
	
	while(ob_front != 1){
		move_forward(dSpeed, dDistance);
		
		// update position on the map
		robot_x = wtom(ot.result.x);
		robot_y = wtom(ot.result.y);
		
		for(i = 0;i < NUM_DIST_SENS;i++){
			if(wb_distance_sensor_get_value(ps[i]) > OCCUPANCE_DIST){
				occupied_cell(robot_x, robot_y, ot.result.theta + angle_offset[i]);
			}
		}
		/* if(ob_front && ob_left){
			turn_right(dSpeed);
		}else if(ob_front && ob_right){
			turn_left(dSpeed);
		} */
	};
	if(ob_front && ob_left){
		turn_right(dSpeed);
	}else if(ob_front && ob_right){
		turn_left(dSpeed);
	}
}