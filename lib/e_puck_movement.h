/*
 * File:          e_puck_movement.h
 * Date:          10.03.2014
 * Description:   Header file for e_puck_movement.c
 * Author:        Stefan Klaus
 * Modifications: V 0.1
 */
 
 

/* Stops the robot */
void stop_robot();
/* move the robot forward a given distance with the given distance */
void move_forward(double dSpeed, double dDist);
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
void UMBmark(double dSpeed, double dDistance); 
/* clockwise part of the UMBenchmark */
void measure_clockWise(double dSpeed, double dDistance);
/* counter clockwise part of the UMBenchmark */
void measure_CounterClockWise(double dSpeed, double dDistance);
/* set the status of the LEDs */
void set_leds(int iActive);

