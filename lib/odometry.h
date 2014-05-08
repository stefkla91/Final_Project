/**
 * File:          odometry.h
 * Date:          14.03.2014
 * Description:   Header file for the odometry.c source file
 * Author:        Stefan Klaus
 * Modifications: V 1.0
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H
 struct odometryTrackStruct { //global odometry struct
	struct {
		float wheel_distance; //data of the e-puck robot
		float wheel_conversion;
	} configuration;
	struct {
		int pos_left_prev; //tempeorary wheel encoder data
		int pos_right_prev;
	} state;
	struct { //X and Y coordinates and rotation theta of the robot
		float x;
		float y;
		float theta;
	} result;
};
/*initiallizes the odometry algorithm*/
int odometry_track_start(struct odometryTrackStruct * ot);
/*start odometry tracking*/
int odometry_track_start_pos(struct odometryTrackStruct * ot, double* dEncPos);
/*collects data needed for odometry_track_step_pos*/
void odometry_track_step(struct odometryTrackStruct * ot);
/*updates the odometry struct*/
void odometry_track_step_pos(struct odometryTrackStruct * ot, double* dEncPos);

#endif