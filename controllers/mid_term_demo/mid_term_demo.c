/*
 * File:          mid_term_demo.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>

#include "../../lib/map_building.h"
#include "../../lib/e_puck_movement.h"

#define TIME_STEP 8 

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
	double dSpeed = 300.0f;
	double dDist = 0.1f;

	wb_robot_init();
	reset();


	//while (wb_robot_step(TIME_STEP*4) != -1) {
		UMBmark(dSpeed, dDist);
		move_forward(dSpeed, 0.3f);
		turn_angle(180.0f, dSpeed);
	//}
	wb_robot_cleanup();

	return 0;
}
