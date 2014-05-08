/**					
 * File:          e_puck_distance_sensors.h
 * Date:          04.04.2014
 * Description:   Header file for e_puck_distance_sensors.h
 * Author:        Stefan Klaus
 * Modifications: V 1.0
 */
 
 /*Initailize the distance sensors*/
 void init_distance_sensors();
 
 /*returns a pointer to an array of distance sensor values*/
 int* get_sensor_data();