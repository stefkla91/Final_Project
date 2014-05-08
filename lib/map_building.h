/**
 * File:          map_building.h
 * Date:          14.03.2014
 * Description:   Header file for the map_building.c source file
 * Author:        Stefan Klaus
 * Modifications: V 1.0
 */
 
 /*initailizes the dispaly*/
void init_display();
/*marks cells as occupied*/
void occupied_cell(int x, int y, float theta);
/*resets the project. Used to initalize everything in the beginning*/
void reset();
/*Main controll loop of the project*/
void run();