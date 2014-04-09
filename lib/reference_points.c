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
#include "map_building.h"

 /**
Sets the reference point in the referencePos struct.
The parameters are a pointer to the referencePos struct and an int.
The int specifies which corner will be set.
1 = lower_left
2 = lower_right
3 = upper_left
4 = upper_right
*/
void setReferencePoint(struct odometryTrackStruct * ot, struct referencePos * ref, int corner){

	if(corner == 1){
		ref->lower_left.x += ot->result.x;
		ref->lower_left.y += ot->result.y;
	}else if(corner == 2){
		ref->lower_right.x = ot->result.x;
		ref->lower_right.y = ot->result.y;
	}else if(corner == 3){
		ref->upper_left.x = ot->result.x;
		ref->upper_left.y = ot->result.y;
	}else if(corner == 4){
		ref->upper_right.x = ot->result.x;
		ref->upper_right.y = ot->result.y;
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
	double dCurPosX = ot->result.x;
	int direction = check_direction(ot->result.theta);

	if(direction == 1){
		if(((ref->upper_left.x <= dCurPosX + dThreshold) || (ref->upper_left.x >= dCurPosX - dThreshold)) && corner == 3){
			setReferencePoint(ot, ref, corner);
		}else if(((ref->upper_right.x <= dCurPosX + dThreshold )|| (ref->upper_right.x >= dCurPosX - dThreshold)) && corner == 4){
			setReferencePoint(ot, ref, corner);
		}
	}else if(direction == 2){
		if(((ref->upper_right.x <= dCurPosX + dThreshold) || (ref->upper_right.x >= dCurPosX - dThreshold)) && corner == 4){
			setReferencePoint(ot, ref, corner);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 2){
			setReferencePoint(ot, ref, corner);
		}
	}else if(direction == 3){
		if(((ref->lower_left.x <= dCurPosX + dThreshold) || (ref->lower_left.x >= dCurPosX - dThreshold)) && corner == 1){
			setReferencePoint(ot, ref, corner);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 2){
			setReferencePoint(ot, ref, corner);
		}
	}else if(direction == 4){
		if(((ref->upper_left.x <= dCurPosX + dThreshold) || (ref->upper_left.x >= dCurPosX - dThreshold)) && corner == 3){
			setReferencePoint(ot, ref, corner);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 1){
			setReferencePoint(ot, ref, corner);
		}
	}
}
/**
This function checks the current robot position with the previous recorded reference position.
If a relation is found, the current encoder position are updated with the saved encoder postions.
The parameters are a pointer to the struct odometrtTrackStruct and refrencePos 
and an int which defines which corner it is. 
The corner is defined in the run() function where this function will be called.
1 = lower_left
2 = lower_right
3 = upper_left
4 = upper_right
*/
void checkReferencePoints(struct odometryTrackStruct * ot, struct referencePos * ref, int corner){
	double dThreshold = 10.0f;
	double *point_dEncPos = get_encoder_positions(); 
	double dCurPosX = ot->result.x;
	int direction = check_direction(ot->result.theta);

	if(ref->upper_left.x && ref->upper_left.y == 0){
		setReferencePoint(ot, ref, corner);
	}else if(ref->upper_right.x && ref->upper_right.y == 0){
		setReferencePoint(ot, ref, corner);
	}else if(ref->lower_left.x && ref->lower_left.y == 0){
		setReferencePoint(ot, ref, corner);
	}else if(ref->lower_left.x && ref->lower_left.y == 0){
		setReferencePoint(ot, ref, corner);
	}

	if(direction == 1){
		if(((ref->upper_left.x <= dCurPosX + dThreshold) || (ref->upper_left.x >= dCurPosX - dThreshold)) && corner == 3){
			ot->result.x = ref->upper_left.x;
			ot->result.y = ref->upper_left.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			updateReferencePoints(ot, ref, corner);
		}else if(((ref->upper_right.x <= dCurPosX + dThreshold )|| (ref->upper_right.x >= dCurPosX - dThreshold)) && corner == 4){
			updateReferencePoints(ot,ref, corner);
			ot->result.x = ref->upper_right.x;
			ot->result.y = ref->upper_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
		}
	}else if(direction == 2){
		if(((ref->upper_right.x <= dCurPosX + dThreshold) || (ref->upper_right.x >= dCurPosX - dThreshold)) && corner == 4){
			ot->result.x = ref->upper_right.x;
			ot->result.y = ref->upper_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			updateReferencePoints(ot,ref, corner);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 2){
			ot->result.x = ref->lower_right.x;
			ot->result.y = ref->lower_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			updateReferencePoints(ot,ref, corner);
		}
	}else if(direction == 3){
		if(((ref->lower_left.x <= dCurPosX + dThreshold) || (ref->lower_left.x >= dCurPosX - dThreshold)) && corner == 1){
			ot->result.x = ref->lower_left.x;
			ot->result.y = ref->lower_left.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			setReferencePoint(ot, ref, corner);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 2){
			ot->result.x = ref->lower_right.x;
			ot->result.y = ref->lower_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1];
			updateReferencePoints(ot,ref, corner); 
		}
	}else if(direction == 4){
		if(((ref->upper_left.x <= dCurPosX + dThreshold) || (ref->upper_left.x >= dCurPosX - dThreshold)) && corner == 3){
			ot->result.x = ref->upper_left.x;
			ot->result.y = ref->upper_left.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			updateReferencePoints(ot,ref, corner);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 1){
			ot->result.x = ref->lower_right.x;
			ot->result.y = ref->lower_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			updateReferencePoints(ot,ref, corner);
		}
	}
}

