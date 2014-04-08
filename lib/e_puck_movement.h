/**
 * File:          e_puck_movement.h
 * Date:          10.03.2014
 * Description:   Header file for e_puck_movement.c
 * Author:        Stefan Klaus
 * Modifications: V 0.3
 */

//#include "odometry.h"


/** Stops the robot */
void stop_robot();
/* move the robot forward a given distance with the given distance */
//void move_forward(double dSped, double dDis, struct odometryTrackStruct * ot);
/* turn 90degrees to the left with the given speed */
void turn_left(double dSpeed);
/* turn 90degrees to the right with the given speed */
void turn_right(double dSpeed);
/* turn the given angle with the given speed */
void turn_angle(double dAngle, double dSpeed); 
/* set the motor speed for the left and right motor */
void set_motor_speed(double dSpeedL, double dSpeedR);
/* return a double with the current encoder position */
double* get_encoder_positions();
/* return a double with the computated odometry information */
double* compute_odometry_data();
/* initialize the UMBenchmark algorithm */
//void UMBmark(double dSpeed, double dDistance); 
/* clockwise part of the UMBenchmark */
//void measure_clockWise(double dSpeed, double dDistance);
/* counter clockwise part of the UMBenchmark */
//void measure_CounterClockWise(double dSpeed, double dDistance);
/* set the status of the LEDs */
void set_leds(int iActive);
/* normalises the angle and stops the robot from moving the strong to one side */
void controll_angle();
/* compares the current heading with the wanted heading and fixes any diversion which surpasses a threshold */
void check_rotation(double cur_rot, double want_rot, double dSpeed);
 