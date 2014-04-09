/**
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
#include <stdio.h>

#include "reference_points.h"
#include "map_building.h"
#include "e_puck_movement.h"
#include "odometry.h"
#include "e_puck_distance_sensors.h"


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
/*#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7 */

//states for the FSM
#define FORWARD 0
#define STOP 1
#define TURNRIGHT 2
#define TURNLEFT 3
#define UTURN 4

#define RTOD(r) ((r) * 180 / M_PI)
#define ANGLE_TOLERANCE 20
#define EAST 0 
#define NORTH 90
#define WEST 180
#define SOUTH 270

/* Definitions */
//Leds
WbDeviceTag led[3];

//Distance sensors and corresponding arrays
/* WbDeviceTag ps[NUM_DIST_SENS]; */

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
/* WbDeviceTag ps[NUM_DIST_SENS];
int ps_value[NUM_DIST_SENS]={0,0,0,0,0,0,0,0}; */
int obstacle[NUM_DIST_SENS]={0,0,0,0,0,0,0,0};
bool ob_front, ob_right, ob_left;

// this is the angle at which the IR sensors are placed on the robot
float angle_offset[NUM_DIST_SENS] = {0.2793, 0.7854, 1.5708, 2.618, -2.618, -1.5708, -0.7854, -0.2793};

//movement
int new_encoder;

//direction
bool north,west,south,east;

//odometry struct
//struct odometryTrackStruct ot;

//starting state for the switch statement
int state = FORWARD;

/**
set booleans for the direction the robot is moving in
1 = north
2 = east
3 = south
4 = west
*/
int check_direction(double d){
	int i = return_angle(d);
	east = false;
	north = false;
	west = false;
	south = false; 
	
	if(i + ANGLE_TOLERANCE >= 360){
		i -= 360; 
	} 
	
	if(EAST < i + ANGLE_TOLERANCE && EAST > i - ANGLE_TOLERANCE){
		east = true;
		return 1;
	}else if(NORTH < i + ANGLE_TOLERANCE && NORTH > i - ANGLE_TOLERANCE){
		north = true;
		return 2;
	}else if(WEST < i + ANGLE_TOLERANCE && WEST > i - ANGLE_TOLERANCE){
		west = true;
		return 3;
	}else if(SOUTH < i + ANGLE_TOLERANCE && SOUTH > i - ANGLE_TOLERANCE){
		south = true;
		return 4; 
	}
}


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
	int i, j;
	
	//initialize the display
	init_display();
	//initialize the distance sensors
	init_distance_sensors(TIME_STEP);

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

/**
run function
*/
void run(struct odometryTrackStruct * ot, struct referencePos * ref){
	int i, it;
	int ps_offset[NUM_DIST_SENS] = {35,35,35,35,35,35,35,35};
	double cur_rot;
	int *point_SensorData;
	
	double dSpeed = 500.0f;
	double dDistance = 0.01f; //0,01
	
	char no[] = "north";
	char ea[] = "east";
	char we[] = "west";
	char so[] = "south";
	
	robot_x = wtom(ot->result.x);
	robot_y = wtom(ot->result.y);
	
	
	point_SensorData = get_sensor_data(NUM_DIST_SENS);
	for(i = 0;i < NUM_DIST_SENS;i++){
		obstacle[i] = point_SensorData[i] - ps_offset[i] > THRESHOLD_DIST;
	}	
	//define boolean for sensor states for cleaner implementation
	bool ob_front = 
	obstacle[0] ||
	obstacle[7];

	bool ob_right = 
	obstacle[2];

	bool ob_left = 
	obstacle[5];
	

	//move_forward(dSpeed, dDistance);
	
	//mark cells as occupied
	wb_display_image_paste(display,background,0,0);
	wb_display_set_color(display,0x000000);
	for(i = 0;i < NUM_DIST_SENS;i++){
		if(point_SensorData[i] > OCCUPANCE_DIST){
			occupied_cell(robot_x, robot_y, ot->result.theta + angle_offset[i]);
		}
	}
	wb_display_image_delete(display,background);
	background = wb_display_image_copy(display,0,0,display_width,display_height);
	
	switch(state){
		case FORWARD:
			check_direction(ot->result.theta);
			cur_rot = return_angle(ot->result.theta);
		 	if(north){
				printf("%s\n", no);
				check_rotation(cur_rot, 90, dSpeed);
				move_forward(dSpeed, dDistance, ot);
			}else if(east){
				printf("%s\n", ea);
				check_rotation(cur_rot, 0, dSpeed);
				move_forward(dSpeed, dDistance, ot);
			}else if(south){
				printf("%s\n", so);
				check_rotation(cur_rot, 270, dSpeed);
				move_forward(dSpeed, dDistance, ot);
			}else if(west){
				printf("%s\n", we);
				check_rotation(cur_rot, 180, dSpeed);
				move_forward(dSpeed, dDistance, ot);
			} 
			//move_forward(dSpeed, dDistance);
			//set_motor_speed(dSpeed, dSpeed);
			 if(ob_front){
				state = STOP;
				}
			break;			
		case STOP:
			stop_robot();
			point_SensorData = get_sensor_data(NUM_DIST_SENS);
			for(i = 0;i < NUM_DIST_SENS;i++){
				obstacle[i] = point_SensorData[i] - ps_offset[i] > THRESHOLD_DIST;
			}	
			//define boolean for sensor states for cleaner implementation
			bool ob_front = 
			obstacle[0] ||
			obstacle[7];

			bool ob_right = 
			obstacle[2];

			bool ob_left = 
			obstacle[5];

			odometry_track_step(ot);
			check_direction(ot->result.theta);
		 	if(ob_front && ob_left && north){
				checkReferencePoints(ot, ref, 3);
				state = TURNRIGHT;
				}
			 else if(ob_front && ob_left){
				if(north){
					checkReferencePoints(ot, ref, 3);
				}else if (south || west){
					checkReferencePoints(ot, ref, 1);
				}
				state = UTURN;
			} 
			else if(ob_front && ob_right && east){
				checkReferencePoints(ot, ref, 2);
				state = TURNLEFT;
			}
			else if(ob_front && ob_right){
					if(north){
					checkReferencePoints(ot, ref, 4);
					}else if (south || east){
					checkReferencePoints(ot, ref, 2);
					}
				state = UTURN;
			}
			else if(ob_front){
				check_direction(ot->result.theta);
				state = UTURN;
				//state = TURNRIGHT;
				}   
			break;	
			
		case TURNRIGHT:
			turn_right(dSpeed);
		//	controll_angle(&ot);
			state = FORWARD;
			break;
		case TURNLEFT:
			turn_left(dSpeed);
			//controll_angle(&ot);
			state = FORWARD;
			break;
		case UTURN:
			if(north){
				printf("%s\n", no);
				turn_left(dSpeed);
				for(it = 0;it < 5;it++){
					if((ob_front && ob_right) || (ob_front && ob_left)){
						// odometry_track_step(ot);
						state = STOP;
						break;
					}
					move_forward(dSpeed, dDistance, ot);
					//mark cells as occupied
					wb_display_image_paste(display,background,0,0);
					wb_display_set_color(display,0x000000);
					for(i = 0;i < NUM_DIST_SENS;i++){
						if(point_SensorData[i] > OCCUPANCE_DIST){
							occupied_cell(robot_x, robot_y, ot->result.theta + angle_offset[i]);
						}
					}
					wb_display_image_delete(display,background);
					background = wb_display_image_copy(display,0,0,display_width,display_height); 
				}
				/* odometry_track_step(ot); */
				/* cur_rot = return_angle(ot->result.theta);
				turn_angle(cur_rot - 270, dSpeed);   */
				turn_left(dSpeed);
				odometry_track_step(ot);
				/* cur_rot = return_angle(ot->result.theta);
				check_rotation(cur_rot, 270, dSpeed);  */
				north = false;
				state = FORWARD;
			}else if(east){
				printf("%s\n", ea);
				turn_right(dSpeed);
				for(it = 0;it < 5;it++){
					if((ob_front && ob_right) || (ob_front && ob_left)){
						// odometry_track_step(ot);
						state = STOP;
						break;
					}
					move_forward(dSpeed, dDistance, ot);
					//mark cells as occupied
					wb_display_image_paste(display,background,0,0);
					wb_display_set_color(display,0x000000);
					for(i = 0;i < NUM_DIST_SENS;i++){
						if(point_SensorData[i] > OCCUPANCE_DIST){
							occupied_cell(robot_x, robot_y, ot->result.theta + angle_offset[i]);
						}
					}
				 	wb_display_image_delete(display,background);
					background = wb_display_image_copy(display,0,0,display_width,display_height); 
				}
				/* odometry_track_step(ot); */
				/* cur_rot = return_angle(ot->result.theta);
				turn_angle(cur_rot - 180, dSpeed);  */
				turn_right(dSpeed);
				odometry_track_step(ot);
				/* cur_rot = return_angle(ot->result.theta);
				check_rotation(cur_rot, 180, dSpeed);  */
				east = false;
				state = FORWARD;
			}else if(south){
				printf("%s\n", so);
				turn_right(dSpeed);
				for(it = 0;it < 5;it++){
					if((ob_front && ob_right) || (ob_front && ob_left)){
						// odometry_track_step(ot);
						state = STOP;
						break;
					}
					//mark cells as occupied
					wb_display_image_paste(display,background,0,0);
					wb_display_set_color(display,0x000000);
					for(i = 0;i < NUM_DIST_SENS;i++){
						if(point_SensorData[i] > OCCUPANCE_DIST){
							occupied_cell(robot_x, robot_y, ot->result.theta + angle_offset[i]);
						}
						
					}
					 wb_display_image_delete(display,background);
					background = wb_display_image_copy(display,0,0,display_width,display_height); 
					
					move_forward(dSpeed, dDistance, ot);
				}
				/* odometry_track_step(ot); */
				/*cur_rot = return_angle(ot->result.theta);
				turn_angle(cur_rot - 90, dSpeed); */
				turn_right(dSpeed);
				odometry_track_step(ot);
				/* cur_rot = return_angle(ot->result.theta);
				check_rotation(cur_rot, 90, dSpeed); */ 
				south = false;
				state = FORWARD;
			}else if(west){
				printf("%s\n", we);
				turn_left(dSpeed);
				for(it = 0;it < 5;it++){
					if((ob_front && ob_right) || (ob_front && ob_left)){
						state = STOP;
						break;
					}
					move_forward(dSpeed, dDistance, ot);
					//mark cells as occupied
					wb_display_image_paste(display,background,0,0);
					wb_display_set_color(display,0x000000);
					for(i = 0;i < NUM_DIST_SENS;i++){
						if(point_SensorData[i] > OCCUPANCE_DIST){
							occupied_cell(robot_x, robot_y, ot->result.theta + angle_offset[i]);
						}
					}
					wb_display_image_delete(display,background);
					background = wb_display_image_copy(display,0,0,display_width,display_height);
				}
				/* odometry_track_step(ot); */
				/* cur_rot = return_angle(ot->result.theta);
				turn_angle(cur_rot - 360, dSpeed); */
			
				turn_left(dSpeed);
				odometry_track_step(ot);
			
			/* cur_rot = return_angle(ot->result.theta);
				check_rotation(cur_rot, 360, dSpeed);  */
				west = false;
				state = FORWARD;
			}
			break;
		 default:
			state = FORWARD; 
	}
	wb_display_set_color(display,0xFF0000);
    wb_display_draw_rectangle(display,robot_x, display_height-robot_y-1,1,1);
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