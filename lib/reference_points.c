/**
 * File:          reference_points.c
 * Date:          14.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: V 0.1
 */
#include "reference_points.h"
#include "odometry.h"
#include "e_puck_movement.h"

 /**
Sets the reference point in the referencePos struct.
The parameters are a pointer to the referencePos struct and an int.
The int specifies which corner will be set.
1 = lower_left
2 = lower_right
3 = upper_left
4 = upper_right
*/
void setReferencePoint(struct referencePos * ref, int corner){
	double *point_dEncPos; 

	point_dEncPos = get_encoder_positions();

	if(corner == 1){
		ref->lower_left.x += point_dEncPos[0];
		ref->lower_left.y += point_dEncPos[1];
	}else if(corner == 2){
		ref->lower_right.x = point_dEncPos[0];
		ref->lower_right.y = point_dEncPos[1];
	}else if(corner == 3){
		ref->upper_left.x = point_dEncPos[0];
		ref->upper_left.y = point_dEncPos[1];
	}else if(corner == 4){
		ref->upper_right.x = point_dEncPos[0];
		ref->upper_right.y = point_dEncPos[1];
	}
}

/**
Checks which refrence point to set.
The parameters are a pointer to the struct odometrtTrackStruct and refrencePos 
and an int which defines which corner it is. 
The corner is defined in the run() function where this function will be called.
1 = lower_left
2 = lower_right
3 = upper_left
4 = upper_right
*/
void updateReferencePoints(struct odometryTrackStruct * ot, struct referencePos * ref, int corner){
	double dThreshold = 10.0;
	double *point_dEncPos;
	double dCurPosX = ot->result.x;

	check_direction(ot->result.theta);
	point_dEncPos = get_encoder_positions();

	if(north){
		if(((ref->upper_left.x <= dCurPosX + dThreshold) || (ref->upper_left.x >= dCurPosX - dThreshold)) && corner == 3){
			setReferencePoint(ref, corner);
		}else if(((ref->upper_right.x <= dCurPosX + dThreshold )|| (ref->upper_right.x >= dCurPosX - dThreshold)) && corner == 4){
			setReferencePoint(ref, corner);
		}
	}else if(east){
		if(((ref->upper_right.x <= dCurPosX + dThreshold) || (ref->upper_right.x >= dCurPosX - dThreshold)) && corner == 4){
			setReferencePoint(ref, corner);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 2){
			setReferencePoint(ref, corner);
		}
	}else if(south){
		if(((ref->lower_left.x <= dCurPosX + dThreshold) || (ref->lower_left.x >= dCurPosX - dThreshold)) && corner == 1){
			ref->lower_left.x += point_dEncPos[0];
			ref->lower_left.y += point_dEncPos[1];
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 2){
			setReferencePoint(ref, corner);
		}
	}else if(west){
		if(((ref->upper_left.x <= dCurPosX + dThreshold) || (ref->upper_left.x >= dCurPosX - dThreshold)) && corner == 3){
			setReferencePoint(ref, corner);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 1){
			setReferencePoint(ref, corner);
		}
	}
}

void checkReferencePoints(odometryTrackStruct * ot, struct referencePos * ref, int corner){
	double dThreshold = 10.0f;
}

