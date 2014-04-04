/**
 * File:          e_puck_distance_sensors.c
 * Date:          04.04.2014
 * Description:   
 * Author:        Stefan Klaus
 * Modifications: V 0.1
 */
 
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include "e_puck_distance_sensors.h"

// 8 IR proximity sensors
#define NUM_DIST_SENS 8
#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7

WbDeviceTag ps[NUM_DIST_SENS];

void init_distance_sensors(int timestep){
	int i;
	char textPS[] = "ps0";
	
	//get the distance sensors
	for (i = 0;i < NUM_DIST_SENS;i++){
		ps[i] = wb_robot_get_device(textPS);
		textPS[2]++;
	}
	
	//enable the distance sensor and light sensor devices
	for(i = 0;i < NUM_DIST_SENS;i++){
		wb_distance_sensor_enable(ps[i], timestep);
	}
}

int* get_sensor_data(){
	int i;
	static int dSensorData[NUM_DIST_SENS] = {0,0,0,0,0,0,0,0};
	
	for(i=0;i<NUM_DIST_SENS;i++){
		dSensorData[i] = (int)wb_distance_sensor_get_value(ps[i]);
	} 
	return dSensorData;
}