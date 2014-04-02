/*
 * File:          odometry.c
 * Date:          14.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: V 0.1
 */
 
#include <webots/differential_wheels.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "odometry.h"
#include "e_puck_movement.h"
#include "errno.h"

#ifndef M_PI
	#define M_PI 3.1415926535897932384626433832795L
#endif

#define WHEEL_RADIUS 0.0206625 // avg. wheel radius of the e-puck 1850.
#define WHEELBASE 0.052
#define ENCODER_RESOLUTION 159.23

float increments_per_tour = 1000.0;
float wheel_radius = 0.0207;
float scaling_factor = 1;
float axis_wheel_ratio = 1.4134;

/**
Initializes the odometry algortihms
*/
int odometry_track_start(struct odometryTrackStruct * ot){
	double* point_dEncPos;
	point_dEncPos = get_encoder_positions();
	return(odometry_track_start_pos(ot, point_dEncPos));
}

/**
Start the odometry tracking
Updates the info in the odometryTrackStruct for the first time
*/
int odometry_track_start_pos(struct odometryTrackStruct * ot, double* dEncPos){
	ot->result.x = 0;
	ot->result.y = 0;
	ot->result.theta = 0;
	
	ot->state.pos_left_prev = dEncPos[0];
	ot->state.pos_right_prev = dEncPos[1];
	
	ot->configuration.wheel_distance = axis_wheel_ratio * scaling_factor * (wheel_radius * 2);
	ot->configuration.wheel_conversion= (wheel_radius * 2) * scaling_factor * M_PI / increments_per_tour;
	
	return 1;
}

void odometry_track_step(struct odometryTrackStruct * ot){
	double* point_dEncPos;
	
	point_dEncPos = get_encoder_positions();
	odometry_track_step_pos(ot,	point_dEncPos);
}

void odometry_track_step_pos(struct odometryTrackStruct * ot, double* dEncPos){
	int delta_pos_left, delta_pos_right;
	float delta_left, delta_right, delta_theta, theta2;
	float delta_x, delta_y;
	
	//calculate the difference in position between the previous and current encoder positions. 
	delta_pos_left = dEncPos[0] - ot->state.pos_left_prev;
	delta_pos_right = dEncPos[1] - ot->state.pos_right_prev;
	
	//calculate the rotation based on the displacement of the stepper motors 
	delta_left = delta_pos_left * ot->configuration.wheel_conversion;
	delta_right = delta_pos_right * ot->configuration.wheel_conversion;
	delta_theta = (delta_right - delta_left) / ot->configuration.wheel_distance;
	
	// calculate the x and y displacement 
	theta2 = ot->result.theta + delta_theta * 0.5;
	delta_x = (delta_left + delta_right) * 0.5 *cosf(theta2);
	delta_y = (delta_left + delta_right) * 0.5 * sinf(theta2);
	
	//update the x, y and theta of the struct
	ot->result.x += delta_x;
	ot->result.y += delta_y;
	ot->result.theta += delta_theta;
	
	if(ot->result.theta > M_PI){
		ot->result.theta -= 2*M_PI;
	}
	if(ot->result.theta < -M_PI){
		ot->result.theta += 2*M_PI;
	}
	
	//save current encoder positions to the global buffer 
	ot->state.pos_left_prev = dEncPos[0];
	ot->state.pos_right_prev = dEncPos[1]; 
}