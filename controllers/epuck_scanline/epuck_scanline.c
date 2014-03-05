/*****************************************
  
  intermediate_lawn_mower
  Author: Rohrer Fabien
  
******************************************/



// Included libraries
#include <webots/robot.h> //obtain main library of webots
#include <webots/distance_sensor.h> // distance sensor library
#include <webots/differential_wheels.h>   //obtain dif. wheels library
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

// Global defines
#define THRESHOLD_DIST 300
#define TIME_STEP 32 // [ms] // time step of the simulation
#define SIMULATION 0 // for robot_get_mode() function
#define REALITY 2    // for robot_get_mode() function
#define LEFT 0        // Left side
#define RIGHT 1       // right side

// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7
WbDeviceTag ps[NB_DIST_SENS];	/* proximity sensors */
int ps_value[NB_DIST_SENS]={0,0,0,0,0,0,0,0};
int ps_offset_sim[NB_DIST_SENS] = {35,35,35,35,35,35,35,35};
int ps_offset_real[NB_DIST_SENS] = {375,158,423,682,447,594,142,360}; // to be modified according to your robot
int obstacle[NB_DIST_SENS]; // will contain a boolean information about obstacles

// wheel
#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.026
#define ENCODER_RESOLUTION 159.23

// FSM
#define FORWARD 0
#define STOP 1
#define UTURN 2
#define TURNRIGHT 3
#define TURNLEFT 4

// motor speeds
int speed[2] = {0,0};
// current state
int state = FORWARD;
// timer emulation : use a counter
int n=0;
int old_encoder=0;

static void reset(void)
{ 
  srand(time(0));
  
  int it;
  
  // get distance sensors
  char textPS[] = "ps0";
  for (it=0;it<NB_DIST_SENS;it++) {
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
  }
  
  // enable distance sensor and light sensor devices
  int i;
  for(i=0;i<NB_DIST_SENS;i++) {
    wb_distance_sensor_enable(ps[i],TIME_STEP);
  }
  
  //enable the encoders
  wb_differential_wheels_enable_encoders(TIME_STEP);
  wb_differential_wheels_set_encoders(0,0);
}

static int run(void) {

  // 0. Preprocessing
  // Obtain the correct offset
  int ps_offset[NB_DIST_SENS] = {0,0,0,0,0,0,0,0};
  int i;
  
  if (wb_robot_get_mode() == SIMULATION) {
    for(i=0;i<NB_DIST_SENS;i++){
      ps_offset[i] = ps_offset_sim[i];
    }
  }
  else {
    for(i=0;i<NB_DIST_SENS;i++){
      ps_offset[i] = ps_offset_real[i];
    }
  }

  // 1. Get the sensors values
  // obstacle will contain a boolean information about a collision
  for(i=0;i<NB_DIST_SENS;i++){
    ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
    obstacle[i] = ps_value[i]-ps_offset[i]>THRESHOLD_DIST;
  }
  
  //define boolean for sensor states for cleaner implementation
  bool ob_front = 
    obstacle[PS_RIGHT_10] ||
    obstacle[PS_LEFT_10];
 
  bool ob_right = 
    obstacle[PS_RIGHT_90];
 
  bool ob_left = 
    obstacle[PS_LEFT_90];
  
  // 2. Compute output values (FSM)
  int new_encoder;
  double d_step, d_meter;
  switch (state) {
    case FORWARD:
      speed[LEFT] = 300;
      speed[RIGHT] = 300;
      if (ob_front) {
        state=STOP;
        n=0;
      }/*
      else if((obstacle[PS_RIGHT_10] ||obstacle[PS_LEFT_10]) && obstacle[PS_LEFT_90]){
        state = TURNRIGHT;
        old_encoder=abs(wb_differential_wheels_get_left_encoder());
      }
      else if((obstacle[PS_RIGHT_10] ||obstacle[PS_LEFT_10]) && obstacle[PS_RIGHT_90]){
        state = TURNLEFT;
        old_encoder=abs(wb_differential_wheels_get_left_encoder());
      }*/
      printf("Forward");
      break;
    case STOP:
      speed[LEFT] = 0;
      speed[RIGHT] = 0;
      n++;
    /*  if (n>40){
        state=UTURN;
        old_encoder=abs(wb_differential_wheels_get_left_encoder());
      }
   */
      if ((ob_front)&& ob_right && ob_left && n>40) {
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
      printf("Stop");
      break;
    case UTURN:
      speed[LEFT] = 150;
      speed[RIGHT] = -150;
      new_encoder=abs(wb_differential_wheels_get_left_encoder());
      d_step = (double) new_encoder-old_encoder;
      d_meter = (double)d_step * WHEEL_RADIUS / ENCODER_RESOLUTION;
      if (d_meter/AXLE_LENGTH>3.14){ //standard valie 3.14
        state=FORWARD;
      }
      printf("U-Turn");
      break;
    case TURNRIGHT:
      speed[LEFT] = 150;
      speed[RIGHT] = -150;
      new_encoder=abs(wb_differential_wheels_get_left_encoder());
      d_step = (double)new_encoder-old_encoder;
      d_meter = (double)d_step*WHEEL_RADIUS / ENCODER_RESOLUTION;
      if(d_meter/AXLE_LENGTH > 1.65){
        state=FORWARD;
      }
      printf("turn right");
      break;
    case TURNLEFT:
      speed[LEFT] = -150;
      speed[RIGHT] = 150;
      new_encoder=abs(wb_differential_wheels_get_right_encoder());
      d_step = (double)new_encoder-old_encoder;
      d_meter = (double)d_step*WHEEL_RADIUS / ENCODER_RESOLUTION;
      if(d_meter/AXLE_LENGTH > 1.65){
        state=FORWARD;
      }
      printf("turn left");
      break;    
    default:
      state=FORWARD;
  }
  /*
  // Use a random number:
  int test = 20.0*rand()/(double)RAND_MAX;
  printf("Random numer = %d\n",test);
  */
  // 3. Send the values to actuators
  wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);

  return TIME_STEP;
}

int main() {
  wb_robot_init();
  
  reset();
  
  /* main loop */
  while(wb_robot_step(TIME_STEP) != -1) {
    run();
  }
  
  wb_robot_cleanup();
  
  return 0;
}
