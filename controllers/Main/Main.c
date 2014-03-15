/*
 * File:          Main.c
 * Date:          15.03.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: v0.1
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/led.h>
#include <math.h>

#include "../../lib/e_puck_movement.h"

#define TIME_STEP 8
//definitions
WbDeviceTag led[3];

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
	double dSpeed = 200.0f;
	double dDistance = 0.20f;
	/* necessary to initialize webots stuff */
	wb_robot_init();

	//emable and initialize the wheel encoders
	wb_differential_wheels_enable_encoders(TIME_STEP*4);
	wb_differential_wheels_set_encoders(0.0f, 0.0f);

	// initialize the LEDs ...
	led[0] = wb_robot_get_device("led0");
	led[1] = wb_robot_get_device("led2");
	led[2] = wb_robot_get_device("led6");
  
  
	//while (wb_robot_step(TIME_STEP*4) != -1) {
		//stop_robot();
		move_forward(dSpeed, dDistance);
	//};

	wb_robot_cleanup();

	return 0;
}
