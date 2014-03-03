/*
 * File:          epuck_scanline.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <stdlib.h>
#include <stdio.h>

//clobal definitions
#define TIME_STEP 64 //timestep in msec
#define LEFT 0 //left motor
#define RIGHT 1 //right motor
#define THRESHOLD_DIST 300 //threshold distance for the sensor


//8 IR proximity sensors
#define NUM_DIST_SENS 8
#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7

//states for the FSM
#define FORWARD 0
#define STOP 1
#define UTURN 2
#define TURNRIGHT 3
#define TURNLEFT 4

//wheel stats
#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.026
#define ENCODER_RESOLUTION 159.23

/**
Global Values
*/
//defines for the ps_sensors
WbDeviceTag ps[NUM_DIST_SENS];
int ps_value[NUM_DIST_SENS]={0,0,0,0,0,0,0,0};
int ps_offset[NUM_DIST_SENS] = {35,35,35,35,35,35,35,35};

//start state for the FSM
int state = FORWARD;

//array with boolean information about ps_sensor values
int obstacle[NUM_DIST_SENS];

//global speed arrays
int speed[2] = {0,0};
int fsm_speed[2] = {0,0};

/**
enables the needed sensor devices 
*/
static void reset(void){
  int it, i;
  
  //get the distance sensors
  char textPS[] = "ps0";
  for (it = 0;it < NUM_DIST_SENS;it++){
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
  }
  
  //enable the distance sensor and light sensor devices
  for(i = 0;i < NUM_DIST_SENS;i++){
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  //enable encoders
  wb_differential_wheels_enable_encoders(TIME_STEP);
}

/**
Function which holds the FSM 
*/
void fsm(){
  int old_encoder = 0, new_encoder, n=0;
  double d_step, d_meter;
    
  //define boolean for sensor states for cleaner implementation
  bool ob_front = 
    obstacle[PS_RIGHT_10] ||
    obstacle[PS_LEFT_10];
 
  bool ob_right = 
    obstacle[PS_RIGHT_90];
 
  bool ob_left = 
    obstacle[PS_LEFT_90];
  
  switch(state){
    case FORWARD:
      fsm_speed[LEFT] = 300;
      fsm_speed[RIGHT] = 300;
      if(ob_front){
        state = STOP;
        n = 0;
      }
      break;
   
   case STOP:
      fsm_speed[LEFT] = 0;
      fsm_speed[RIGHT] = 0;
      n++;
      if (ob_front&& ob_right && ob_left && n>40) {
        state=UTURN;
        old_encoder=abs(wb_differential_wheels_get_left_encoder());
      }
      else if(ob_front && ob_left && n>40){
        state = TURNRIGHT;
        old_encoder=abs(wb_differential_wheels_get_left_encoder());
      }
      else if(ob_front && ob_right && n>40){
        state = TURNLEFT;
        old_encoder=abs(wb_differential_wheels_get_left_encoder());
      }
      else{
        state = UTURN;
        old_encoder=abs(wb_differential_wheels_get_left_encoder());
      }
      break;
      
   case UTURN:
      fsm_speed[LEFT] = 150;
      fsm_speed[RIGHT] = -150;
      new_encoder=abs(wb_differential_wheels_get_left_encoder());
      d_step = (double) new_encoder-old_encoder;
      d_meter = (double)d_step * WHEEL_RADIUS / ENCODER_RESOLUTION;
      if (d_meter/AXLE_LENGTH>3.14){ //standard value 3.14
        state=FORWARD;
      }
      break;
    
    case TURNRIGHT:
      fsm_speed[LEFT] = 150;
      fsm_speed[RIGHT] = -150;
      new_encoder=abs(wb_differential_wheels_get_left_encoder());
      d_step = (double)new_encoder-old_encoder;
      d_meter = (double)d_step*WHEEL_RADIUS / ENCODER_RESOLUTION;
      if(d_meter/AXLE_LENGTH > 1.65){
        state=FORWARD;
      }
      break;
   
    case TURNLEFT:
      fsm_speed[LEFT] = -150;
      fsm_speed[RIGHT] = 150;
      new_encoder=abs(wb_differential_wheels_get_right_encoder());
      d_step = (double)new_encoder-old_encoder;
      d_meter = (double)d_step*WHEEL_RADIUS / ENCODER_RESOLUTION;
      if(d_meter/AXLE_LENGTH > 1.65){
        state=FORWARD;
      }
      break;    
   
    default:
      state=FORWARD;
  }
  wb_differential_wheels_set_speed(fsm_speed[LEFT], fsm_speed[RIGHT]);
}

/**
The run method which calls the other modules
*/
static int run(void){
  int run_speed[2] = {0,0};
  int i;
  
  //gets the ps_sensor values and applies the offset and thresshold
    for(i=0;i<NUM_DIST_SENS;i++){
      ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
      obstacle[i] = ps_value[i]-ps_offset[i]>THRESHOLD_DIST;
    }
    
   /* wb_differential_wheels_set_encoders(0,0);
    if(abs(wb_differential_wheels_get_left_encoder() < 1234)){
      fsm();
    }
    else{
      run_speed[LEFT] = 0;
      run_speed[RIGHT] = 0;
    }
    */
    fsm();
    //set actuators
    wb_differential_wheels_set_speed(
      speed[LEFT] + fsm_speed[LEFT] + run_speed[LEFT],
      speed[RIGHT] + fsm_speed[RIGHT] + run_speed[RIGHT]
      );
  return TIME_STEP;
}

/**
Main method  
*/
int main(){
  wb_robot_init();
  
  reset();
  while(wb_robot_step(TIME_STEP) != 1){
    run();
  }
  wb_robot_cleanup();
  
  return 0;
}

