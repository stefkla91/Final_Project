/**
 * File:          reference_points.c
 * Date:          14.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: V 0.1
 */
#include <webots/robot.h>
#include <math.h>	
#include "reference_points.h"
#include "odometry.h"
#include "e_puck_movement.h"
#include "map_building.h"
#include <stdio.h>

#define ANGLE_TOLERANCE 20
 #define EAST 0 
#define NORTH 90
#define WEST 180
#define SOUTH 270
//direction
bool north,west,south,east;

/**
set booleans for the direction the robot is moving in
1 = north
2 = east
3 = south
4 = west
*/
void checkDirecrection(double d){
	int i = return_angle(d);
	east = false;
	north = false;
	west = false;
	south = false; 
	//int result = 0;
	if(i + ANGLE_TOLERANCE >= 360){
		i -= 360; 
	} 
	
	if(EAST < i + ANGLE_TOLERANCE && EAST > i - ANGLE_TOLERANCE){
		east = true;
	//	result = 1;
	}else if(NORTH < i + ANGLE_TOLERANCE && NORTH > i - ANGLE_TOLERANCE){
		north = true;
	//	result = 2;
	}else if(WEST < i + ANGLE_TOLERANCE && WEST > i - ANGLE_TOLERANCE){
		west = true;
	//	result = 3;
	}else if(SOUTH < i + ANGLE_TOLERANCE && SOUTH > i - ANGLE_TOLERANCE){
		south = true;
	//	result = 4; 
	}
	//return result;
}
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
	char text[] = "Saving";
	printf("%s\n", text);
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
	double dThreshold = 20.0;
	double dCurPosX = ot->result.x;
	int direction;
	//int direction = checkDirecrection(ot->result.theta);
	checkDirecrection(ot->result.theta);

	if(north){
		direction = 1;
	}else if(east){
		direction = 2;
	}else if(south){
		direction = 3;
	}else if(west){
		direction = 4;
	}

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
	double dThreshold = 20.0f;
	double *point_dEncPos = get_encoder_positions(); 
	double dCurPosX = ot->result.x;
	int direction;
	//int direction = checkDirecrection(ot->result.theta);
	checkDirecrection(ot->result.theta);

	if(north){
		direction = 1;
	}else if(east){
		direction = 2;
	}else if(south){
		direction = 3;
	}else if(west){
		direction = 4;
	}

	if((ref->upper_left.x == 0.00 && ref->upper_left.y == 0.00) && corner == 3){
		setReferencePoint(ot, ref, corner);
	}else if((ref->upper_right.x == 0.00 && ref->upper_right.y == 0.00)&& corner == 4){
		setReferencePoint(ot, ref, corner);
	}else if((ref->lower_left.x == 0.00 && ref->lower_left.y == 0.00)&& corner == 1){
		setReferencePoint(ot, ref, corner);
	}else if((ref->lower_right.x == 0.00 && ref->lower_right.y == 0.00)&& corner == 2){
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

