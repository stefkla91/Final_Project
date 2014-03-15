/*
 * File:          my_controller_movement.c
 * Date:          13.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: v 0.1
 */
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
//#include <webots/accelerometer.h>
#include <webots/led.h>
#include <math.h>
#include <webots/display.h> 
//include odometry file
//#include "../../lib/odometry.h" 

#ifndef M_PI
	#define M_PI 3.1415926535897932384626433832795L
#endif

#define TIME_STEP 8 
#define WHEEL_RADIUS 0.0206625 // avg. wheel radius of the e-puck 1850.
#define WHEELBASE 0.052
#define ENCODER_RESOLUTION 159.23
#define INCREMENT_STEP 1000 //steps of the motor for a whole rotation orig REV_STEP
#define STEP_TOLERANCE 6.0
#define RANGE (1024 / 2)
#define MIN_DIST 20.0f //minimum distance before slowing down
#define MIN_SPEED 10.0f //speed when slowing down
#define NUMTOURNAMENTS 1 //5

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
#define SIMULATION 0 // for robot_get_mode() function
//map definitions
#define MAP_SIZE 70
#define CELL_SIZE 0.015 //[m] width and hight 

//defines for the ps_sensors
WbDeviceTag ps[NUM_DIST_SENS];
int ps_value[NUM_DIST_SENS]={0,0,0,0,0,0,0,0};
int ps_offset_sim[NUM_DIST_SENS] = {35,35,35,35,35,35,35,35};
int obstacle[NUM_DIST_SENS]; //array with boolean information about ps_sensor values
bool ob_front, ob_right, ob_left;

//display
WbDeviceTag display;
WbImageRef background;
int display_width;
int display_height;

//movement
double d_meter, d_step;
int new_encoder;

//instantiat odometry track and goto structures
struct sOdometryTrack ot;

//instantiate map structures
int map[MAP_SIZE][MAP_SIZE];

// the robot position on the map
int robot_x = MAP_SIZE / 2;
int robot_y = MAP_SIZE / 2;

// this is the angle at which the IR sensors are placed on the robot
float angle_offset[NUM_DIST_SENS] = {0.2793, 0.7854, 1.5708, 2.618, -2.618, -1.5708, -0.7854, -0.2793};

/////////////////////////////

double dCurSpeed[2] = {0.0f, 0.0f}; // global buffer to store the current speed values. orig:dMSpeed
double dPrevEncPos[2] = {0.0f, 0.0f}; // global buffer to save the previous encoder postions.
double *point_dOdometryData; // global buffer to save the current odometry data.
WbDeviceTag led[3]; // LEDs.

// motion control methods:
static void stop_robot();
static void move_forward(double dSpeed, double dDist);
static void turn_left(double dSpeed);
static void turn_right(double dSpeed);
static void turn_angle(double dAngle, double dSpeed); 
static void set_motor_speed(double dSpeedL, double dSpeedR);
static double* get_encoder_positions();

// odometry:
static double* compute_odometry_data();

// odometry UMBmark:
static void UMBmark(double dSpeed, double dDistance); //calibration 
static void measure_clockWise(double dSpeed, double dDistance);
static void measure_CounterClockWise(double dSpeed, double dDistance);

// LEDs:
static void set_leds(int iActive);

/* Occupany functions before the main, movement after */

/**
 * Initiate the display with a white color
 */
static void init_display(){
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

/*
 * Set the coresponding cell to 1 (occuM_PIed)
 * and display it
 */
void occuM_PIed_cell(int x, int y, float theta){
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
static void reset(void){
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
}

static int run(void){
	int i;
	
	//define variables
	double dSpeed = 200.0f;
	double dDistance = 0.20f;
	
	int ps_offset[NUM_DIST_SENS] = {0,0,0,0,0,0,0,0};
	
	if (wb_robot_get_mode() == SIMULATION) {
		for(i=0;i<NUM_DIST_SENS;i++){
			ps_offset[i] = ps_offset_sim[i];
		}
	}
/* 	// for each sensor if the value is above threshold we declare the
	// corresponding cell as occuM_PIed
	wb_display_image_paste(display,background,0,0);
	wb_display_set_color(display,0x000000);
	for (i = 0; i < NUM_DIST_SENS; i++) {
		if (wb_distance_sensor_get_value(ps[i]) > OCCUPANCE_DIST) {
			occuM_PIed_cell(robot_x, robot_y, ot.result.theta + angle_offset[i]);
		}
	}
	wb_display_image_delete(display,background);
	background = wb_display_image_copy(display,0,0,display_width,display_height);
	*/
	// 1. Get the sensors values
	// obstacle will contain a boolean information about a collision
	for(i=0;i<NUM_DIST_SENS;i++){
		ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
		obstacle[i] = ps_value[i]-ps_offset[i]>THRESHOLD_DIST;
	} 

	//define boolean for sensor states for cleaner implementation
	bool ob_front = 
	obstacle[PS_RIGHT_10] ||
	obstacle[PS_LEFT_10];

	bool ob_right = 
	obstacle[PS_RIGHT_90];

	bool ob_left = 
	obstacle[PS_LEFT_90];
	
	//move_forward(4, 0.25);
	if(ob_front && ob_left){
		turn_right(dSpeed);
	} else if(ob_front && ob_right){
		turn_left(dSpeed);
	} else if(ob_front){
		turn_right(dSpeed);
	} else {
		move_forward(dSpeed, dDistance);
	}
	
	/* odometry_track_step(&ot);
		
	// update position on the map
	robot_x = wtom(ot.result.x);
	robot_y = wtom(ot.result.y);
	
	wb_display_set_color(display,0xFF0000);
	wb_display_draw_rectangle(display,robot_x, display_height-robot_y-1,1,1); */
	
	return TIME_STEP;
}


int main(int argc, char *argv[]){
	int i;
	
	//init Webots
	wb_robot_init();
	reset();
	//emable and initialize the wheel encoders
	wb_differential_wheels_enable_encoders(TIME_STEP*4);
    wb_differential_wheels_set_encoders(0.0f, 0.0f);
	
	// initialize the LEDs ...
	led[0] = wb_robot_get_device("led0");
	led[1] = wb_robot_get_device("led2");
	led[2] = wb_robot_get_device("led6");
	
	init_display();
	odometry_track_start(&ot);
	ot.result.x = 0.008;
	ot.result.y = 0.008;
	ot.result.theta = 1.5731;
	while (wb_robot_step(TIME_STEP) != -1) {
//	stop_robot(); // precaution ...
	run();
	//initalizes tracking and goto strucutres
	/* odometry_track_step(&ot);
		
	// update position on the map
	robot_x = wtom(ot.result.x);
	robot_y = wtom(ot.result.y);
	
	wb_display_set_color(display,0xFF0000);
	wb_display_draw_rectangle(display,robot_x, display_height-robot_y-1,1,1); */
/* 	// start the UMBmark method ...
	UMBmark(dSpeed, dDistance);
	// deactivate the LEDs - show that the process is finished now ...
	set_leds(0);
	//}
	for (i=0;i<6;i++){
		move_forward(4,0.25);
		turn_left(4);
		move_forward(4,0.25);
		turn_left(4);
		move_forward(4,0.25);
		turn_left(4);
		move_forward(4,0.25);
		turn_left(4);
		move_forward(4,0.25);
		turn_right(4);
		move_forward(4,0.25);
		turn_right(4);
		move_forward(4,0.25);
		turn_right(4);
		move_forward(4,0.25);
		turn_right(4);
	} */
	}
	//clean up
	wb_robot_cleanup();
	return 0;	
}
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
void move_forward(double dSpeed, double dDist){
	double dStepCount = 0.0f;
	double dStopPosLeft = 0.0f;
	double dStopPosRight = 0.0f;
	double *point_dEncPos;	
	if((dDist > 0.0f) && (dSpeed > 0.0f)){
		//calculate the number of steps
		dStepCount = (INCREMENT_STEP/(M_PI * WHEEL_RADIUS * 2.0f)) * dDist;
		
		/*read the current encoder positions of both wheels and
		calculate the encoder positions when the robot has to
		stop at the given distance ... */
		point_dEncPos = get_encoder_positions();
		
		dStopPosLeft = point_dEncPos[0] + dStepCount;
		dStopPosRight = point_dEncPos[1] + dStepCount;
		
		//compute odometry data
		point_dOdometryData = compute_odometry_data();
		
		//set speed
		set_motor_speed(dSpeed, dSpeed);
		
		while((point_dEncPos[0] < dStopPosLeft) && (point_dEncPos[1] < dStopPosRight)){
			///////////////////////////////////////////////
			//get odometry data
			point_dOdometryData = compute_odometry_data();
			
			//map the position on the map
			odometry_track_step(&ot);
		
			// update position on the map
			robot_x = wtom(ot.result.x);
			robot_y = wtom(ot.result.y); 
			
			wb_display_set_color(display,0xFF0000);
			wb_display_draw_rectangle(display,robot_x, display_height-robot_y-1,1,1);
			/////////////////////////
			
			int ps_offset[NUM_DIST_SENS] = {0,0,0,0,0,0,0,0};
			int i;
			// for each sensor if the value is above threshold we declare the
			// corresponding cell as occuM_PIed
			wb_display_image_paste(display,background,0,0);
			wb_display_set_color(display,0x000000);
			for (i = 0; i < NUM_DIST_SENS; i++) {
				if (wb_distance_sensor_get_value(ps[i]) > OCCUPANCE_DIST) {
					occuM_PIed_cell(robot_x, robot_y, ot.result.theta + angle_offset[i]);
				}
			}
			wb_display_image_delete(display,background);
			background = wb_display_image_copy(display,0,0,display_width,display_height);
			
			///////////////////////////////////////////////////////////
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
	
	wb_robot_step(TIME_STEP);
}

/**
Function to turn left
*/
void turn_left(double dSpeed){
	/*turning 100 degrees actually turn the robot ~90 degrees.
	most likely because the wheel radius I have found does not match the 
	one in the simulator*/
	turn_angle(-100.0f, dSpeed); 
}
/**
Function to turn right
*/
void turn_right(double dSpeed){
	/*turning 100 degrees actually turn the robot ~90 degrees.
	most likely because the wheel radius I have found does not match the 
	one in the simulator*/
	turn_angle(100.0f, dSpeed);
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
		dStepCount = (INCREMENT_STEP * WHEELBASE)/(dFactor * WHEEL_RADIUS * 2.0f);
		
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

		// turn left the robot ...
		set_motor_speed(-dSpeed, dSpeed);

		while(  (point_dEncPos[0] > dStopPosLeft) &&(point_dEncPos[1] < dStopPosRight)  ) {

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
void UMBmark(double dSpeed, double dDistance){
	//activate leds to show calibration has started
	set_leds(1);
	
	//measure and calibrate the values of the odometry
	measure_clockWise(dSpeed, dDistance);
	
	measure_CounterClockWise(dSpeed, dDistance);
	
	stop_robot();
}
/**
Function to measure the movement accuracy by driving
a clockwise square.
This is part of the UMBmark algorithm
*/
void measure_clockWise(double dSpeed, double dDistance){
	int i, j;
	
	for(i = 0;i < NUMTOURNAMENTS; i++){
		//compute odometry data
		point_dOdometryData = compute_odometry_data();
		for(j = 0;j < 4;j++){
			move_forward(dSpeed, dDistance);
			turn_right(dSpeed);
		}
		wb_robot_step(TIME_STEP);
	}
	
	//actualize the odometry values
	point_dOdometryData = compute_odometry_data();
}

/**
Function to measure the movement accuracy by driving
a counter-clockwise square.
This is part of the UMBmark algorithm
*/
void measure_CounterClockWise(double dSpeed, double dDistance){
	int i, j; 
	
	//get the odometry data
	point_dOdometryData = compute_odometry_data();
	
	//turn the robot right for moving the same square counter clock wise
	turn_right(dSpeed);
	
	for(i = 0;i < NUMTOURNAMENTS;i++){
		point_dOdometryData = compute_odometry_data();
		
		for(j = 0;j < 4; j++){
			move_forward(dSpeed, dDistance);
			turn_left(dSpeed);
		}
		wb_robot_step(TIME_STEP);
	}
	
	//actualize the odometry values
	point_dOdometryData = compute_odometry_data();
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