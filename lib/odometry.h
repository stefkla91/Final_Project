/**
 * File:          odometry.h
 * Date:          14.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: V 0.2
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H
 struct odometryTrackStruct {
	struct {
		float wheel_distance;
		float wheel_conversion;
	} configuration;
	struct {
		int pos_left_prev;
		int pos_right_prev;
	} state;
	struct {
		float x;
		float y;
		float theta;
	} result;
};
 
int odometry_track_start(struct odometryTrackStruct * ot);
int odometry_track_start_pos(struct odometryTrackStruct * ot, double* dEncPos);
void odometry_track_step(struct odometryTrackStruct * ot);
void odometry_track_step_pos(struct odometryTrackStruct * ot, double* dEncPos);

#endif