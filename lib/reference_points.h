/**
 * File:          reference_points.h
 * Date:          09.04.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: V 0.2
 */
#include "odometry.h"
#ifndef REFERENCE_POINT_H
#define REFERENCE_POINT_H
 struct referencePos {
 	struct {
 		float x;
 		float y;
 	} lower_left;
 	struct {
 		float x;
 		float y;
 	} lower_right;
 	struct {
 		float x;
 		float y;
 	} upper_left;
 	struct {
		float x;
 		float y;
 	} upper_right;
 };

/** Function to set the reference point in the struct*/
void set_reference_point(struct odometryTrackStruct * ot, struct referencePos * ref, int corner);

/** 
Function to compare the current location with the location saved in the reference points
Calls the set_reference_point function the first time a refernce point is found
*/
void check_reference_points(struct odometryTrackStruct * ot, struct referencePos * ref);

#endif