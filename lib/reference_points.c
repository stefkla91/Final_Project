/**
 * File:          reference_points.c
 * Date:          14.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: V 0.3
 */
#include <webots/robot.h>
#include <math.h>	
#include "functions.h"
#include "reference_points.h"
#include "odometry.h"
#include "e_puck_distance_sensors.h"
#include "e_puck_movement.h"
#include "map_building.h"
#include <stdio.h>

#define NUM_DIST_SENS 8
#define THRESHOLD_DIST 100


 /**
Sets the reference point in the referencePos struct.
The parameters are a pointer to the referencePos struct and an int.
The int specifies which corner will be set.
1 = lower_left
2 = lower_right
3 = upper_left
4 = upper_right
*/
void set_reference_point(struct odometryTrackStruct * ot, struct referencePos * ref, int corner){
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
void check_reference_points(struct odometryTrackStruct * ot, struct referencePos * ref){
	double dThreshold = 20.0f;
	int i;
	double *point_dEncPos = get_encoder_positions(); 
	double dCurPosX = ot->result.x;
	int *point_SensorData;
	int corner = 0;
	int obstacle[NUM_DIST_SENS]={0,0,0,0,0,0,0,0};
	int ps_offset[NUM_DIST_SENS] = {35,35,35,35,35,35,35,35};
	int direction = check_direction(ot->result.theta);
	bool ob_front, ob_left, ob_right;
	char message[] = "Resetting";

/*	get distance sensor data*/
	point_SensorData = get_sensor_data(NUM_DIST_SENS);
	for(i = 0;i < NUM_DIST_SENS;i++){
		obstacle[i] = point_SensorData[i] - ps_offset[i] > THRESHOLD_DIST;
	}	

	ob_front  = 
	obstacle[0] ||
	obstacle[7];

	ob_right = 
	obstacle[2];

	ob_left = 
	obstacle[5];

	/*find out what corner it is */
	//north 
	if(direction == 1){
		if(ob_front && ob_left){
			corner = 3;
		}else if(ob_front && ob_right){
			corner = 4;
		}
	}

	//east
	if(direction == 2){
		if(ob_front && ob_left){
			corner = 4;
		}else if(ob_front && ob_right){
			corner = 2;
		}
	}

	//south
	if(direction == 3){
		if(ob_front && ob_left){
			corner = 2;
		}else if(ob_front && ob_right){
			corner = 1;
		}
	}

	//west
	if(direction == 4){
		if(ob_front && ob_left){
			corner = 1;
		}else if(ob_front && ob_right){
			corner = 3;
		}
	}

	/*Check if the current corner is set to 0, if so force update*/
	if(corner == 1){
		if((ref->lower_left.x == 0.00) && (ref->lower_left.y == 0.00)){
			set_reference_point(ot, ref, corner);
			return;
		}
	}else if(corner == 2){
		if((ref->lower_right.x == 0.00) && (ref->lower_right.y == 0.00)){
			set_reference_point(ot, ref, corner);
			return;
		}
	}else if(corner == 3){
		if((ref->upper_left.x == 0.00) && (ref->upper_left.y == 0.00)){
			set_reference_point(ot, ref, corner);
			return;
		}
	}else if(corner == 4){
		if((ref->upper_right.x == 0.00) && (ref->upper_right.y == 0.00)){
			set_reference_point(ot, ref, corner);
			return;
		}
	}

	/*If the corner is set updated the odometery information of the robot */
	if(direction == 1){ //north 
		if(((ref->upper_left.x <= dCurPosX + dThreshold) || (ref->upper_left.x >= dCurPosX - dThreshold)) && corner == 3){
			ot->result.x = ref->upper_left.x;
			ot->result.y = ref->upper_left.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			printf("%s\n", message);
		}else if(((ref->upper_right.x <= dCurPosX + dThreshold )|| (ref->upper_right.x >= dCurPosX - dThreshold)) && corner == 4){
			ot->result.x = ref->upper_right.x;
			ot->result.y = ref->upper_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			printf("%s\n", message);
		}
	}else if(direction == 2){//east 
		if(((ref->upper_right.x <= dCurPosX + dThreshold) || (ref->upper_right.x >= dCurPosX - dThreshold)) && corner == 4){
			ot->result.x = ref->upper_right.x;
			ot->result.y = ref->upper_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			printf("%s\n", message);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 2){
			ot->result.x = ref->lower_right.x;
			ot->result.y = ref->lower_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			printf("%s\n", message);
		}
	}else if(direction == 3){ //south
		if(((ref->lower_left.x <= dCurPosX + dThreshold) || (ref->lower_left.x >= dCurPosX - dThreshold)) && corner == 1){
			ot->result.x = ref->lower_left.x;
			ot->result.y = ref->lower_left.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			printf("%s\n", message);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 2){
			ot->result.x = ref->lower_right.x;
			ot->result.y = ref->lower_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1];
			printf("%s\n", message);
		}
	}else if(direction == 4){ //west
		if(((ref->upper_left.x <= dCurPosX + dThreshold) || (ref->upper_left.x >= dCurPosX - dThreshold)) && corner == 3){
			ot->result.x = ref->upper_left.x;
			ot->result.y = ref->upper_left.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			printf("%s\n", message);
		}else if(((ref->lower_right.x <= dCurPosX + dThreshold) || (ref->lower_right.x >= dCurPosX - dThreshold)) && corner == 1){
			ot->result.x = ref->lower_right.x;
			ot->result.y = ref->lower_right.y;
			ot->state.pos_left_prev = point_dEncPos[0];
			ot->state.pos_right_prev = point_dEncPos[1]; 
			printf("%s\n", message);
		}
	}
}

