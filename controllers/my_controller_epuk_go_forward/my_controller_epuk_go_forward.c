/*
author: Stefan Klaus

*/



#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/led.h>
#include <math.h>

#ifndef M_PI
	#define M_PI 3.1415926535897932384626433832795L
#endif


#define TIME_STEP 8 
#define WHEEL_RADIUS 0.0206625 // avg. wheel radius of the e-puck 1850.
#define WHEELBASE 0.052
#define ENCODER_RESOLUTION 159.23
#define REV_STEP 1000 
#define STEP_TOLERANCE 6.0
#define RANGE (1024 / 2)
#define MIN_DIST 20.0f
#define MIN_SPEED 10.0f
#define NTOURNAMENTS 1 //5


double dMSpeed[2] = {0.0f, 0.0f}; // global buffer to store the current speed values.
double dPrevEncPos[2] = {0.0f, 0.0f}; // global buffer to save the previous encoder postions.
double *p_dOdometryData; // global buffer to save the current odometry data.
WbDeviceTag led[3]; // LEDs.

// motion control methods:
static void stop_robot();
static void move_forward(double dSpeed, double dDist);
static void turn_left(double dSpeed);
static void turn_right(double dSpeed);
static void turn_angle(double dAngle, double dSpeed); 
static void set_motor_speed(double dSpeedL, double dSpeedR);
static double* get_encoder_positions();

// odometry:
static double* compute_odometry_data();

// odometry calibration:
static void UMBmark(double dSpeed, double dDistance);
static void measure_cw(double dSpeed, double dDistance);
static void measure_ccw(double dSpeed, double dDistance);

// LEDs:
static void set_leds(int iActive);


int main(int argc, char *argv[]) {
int i;
  // define variables ...
  double dSpeed = 200.0f;
  double dDistance = 0.20f;
  
  // initialize Webots ...
  wb_robot_init();
  // enable and initialize the wheel devices ...
  wb_differential_wheels_enable_encoders(TIME_STEP*4);
  wb_differential_wheels_set_encoders(0.0f, 0.0f);
  
  // initialize the LEDs ...
  // (note: the command "sprintf" is for crosscompiling not possible).
  led[0] = wb_robot_get_device("led0");
  led[1] = wb_robot_get_device("led2");
  led[2] = wb_robot_get_device("led6");
  
  //while (wb_robot_step(TIME_STEP) != -1) {
    stop_robot(); // precaution ...
    
    // start the calibration method ...
    UMBmark(dSpeed, dDistance);
    // deactivate the LEDs - show that the process is finished now ...
    set_leds(0);
  //}
  for (i=0;i<6;i++)
  {
    move_forward(4,0.25);
    turn_left(4);
    move_forward(4,0.25);
    turn_left(4);
    move_forward(4,0.25);
    turn_left(4);
    move_forward(4,0.25);
    turn_left(4);
    move_forward(4,0.25);
    turn_right(4);
    move_forward(4,0.25);
    turn_right(4);
    move_forward(4,0.25);
    turn_right(4);
    move_forward(4,0.25);
    turn_right(4);
  }
  // necessary for the real robot
  // (else the robot will never stop)
  for(;;) { stop_robot(); } 
  // finally ...
  wb_robot_cleanup();
  return 0;
}


void stop_robot() {
  dMSpeed[0] = 0.0f;
  dMSpeed[1] = 0.0f;
  wb_differential_wheels_set_speed(dMSpeed[0], dMSpeed[1]);
}

void move_forward(double dSpeed, double dDist) {
  double dStepCount = 0.0f;
  double dStopPosLeft = 0.0f;
  double dStopPosRight = 0.0f;
  double *p_dEncPos;
  
  if( (dDist > 0.0f) && (dSpeed > 0.0f) ) {    
    // calculate the number of step counts ...
    dStepCount = (REV_STEP/(M_PI*WHEEL_RADIUS*2.0f))*dDist;

    // read the current encoder positions of both wheels and
    // calculate the encoder positions when the robot has to
    // stop at the given distance ...
    p_dEncPos = get_encoder_positions();
    
    dStopPosLeft = p_dEncPos[0] + dStepCount;
    dStopPosRight = p_dEncPos[1] + dStepCount;
    
    // compute the current odometry data ...
    p_dOdometryData = compute_odometry_data();
    
    // set the speed of both wheels (move forward) ...
    set_motor_speed(dSpeed, dSpeed);

    // wait until the robot has reached the calculated positions of the encoders ...
    
    /*while(  (dStopPosLeft + STEP_TOLERANCE < p_dEncPos[0]) ||    // beyond the distance (*)...
            (dStopPosLeft - STEP_TOLERANCE > p_dEncPos[0]) ||    // short behind the distance (**)...
            (dStopPosRight + STEP_TOLERANCE < p_dEncPos[1]) ||  // (*)
            (dStopPosRight - STEP_TOLERANCE > p_dEncPos[1]) ) { // (**) */
    while(  (p_dEncPos[0] < dStopPosLeft) && 
            (p_dEncPos[1] < dStopPosRight) ) {
              // actualize the odometry data ...
              p_dOdometryData = compute_odometry_data();
              // actualize the wheel encoder position values ...
              p_dEncPos = get_encoder_positions();
              // slow down the speed if the robot is very close to the end (increases the precision)...
              if( fabs(dStopPosLeft - p_dEncPos[0]) <= MIN_DIST ) { set_motor_speed(MIN_SPEED, MIN_SPEED); }
    }
  }  
  // the given distance is reached, so stop the robot ...
  stop_robot();
  
  // compute the odometry data ...
  p_dOdometryData = compute_odometry_data();
  // request the simulator to perform a simulation step [ms] ...
  wb_robot_step(TIME_STEP); 
}

void turn_left(double dSpeed) { //orig -90.0f
  turn_angle(-100.0f, dSpeed);
}

void turn_right(double dSpeed) {//orig 90.0f
  turn_angle(100.0f, dSpeed);
}

void turn_angle(double dAngle, double dSpeed) {
  double dFactor = 0.0f;
  double dStepCount = 0.0f;
  double dStopPosLeft = 0.0f;
  double dStopPosRight = 0.0f;
  double *p_dEncPos;
  
  if( (dAngle != 0.0f) && (dSpeed > 0.0f) ) {
    // calculate the turn factor ...
    dFactor = fabs(360.0f/dAngle);
    
    // calculate the number of step counts for the wheel rotations ...
    dStepCount = (REV_STEP*WHEELBASE)/(dFactor*WHEEL_RADIUS*2.0f); // e-puck
    //dStepCount = ((WHEELBASE*M_PI*abs(angle))/360.0f)*STEP // Khepera
    
    // read the current encoder positions of the wheels ...
    p_dEncPos = get_encoder_positions();
    
    if(dAngle > 0) { // turn right ...
      // calculate the encoder positions ...
      dStopPosLeft = p_dEncPos[0] + dStepCount;
      dStopPosRight = p_dEncPos[1] - dStepCount;
      
      // compute the current odometry data ...
      p_dOdometryData = compute_odometry_data();

      // turn the robot by setting the different wheel speeds ...
      set_motor_speed(dSpeed, -dSpeed);
      
      // turn right and wait until the robot has reached the 
      // calculated positions of the encoders ...
      while(  (p_dEncPos[0] < dStopPosLeft) && 
              (p_dEncPos[1] > dStopPosRight)  ) {
                // actualize the odometry data ...
                p_dOdometryData = compute_odometry_data();
                // actualize the wheel encoder position values ...
                p_dEncPos = get_encoder_positions();
                // if the robot is very close to the endpoinnt slow down the speed (increases the precision)...
                if( fabs(dStopPosLeft - p_dEncPos[0]) <= MIN_DIST ) { set_motor_speed(MIN_SPEED, -MIN_SPEED); }
      }
    } else { // turn left ...
      dStopPosLeft = p_dEncPos[0] - dStepCount;
      dStopPosRight = p_dEncPos[1] + dStepCount; 

      // turn left the robot ...
      set_motor_speed(-dSpeed, dSpeed);

      while(  (p_dEncPos[0] > dStopPosLeft) &&
              (p_dEncPos[1] < dStopPosRight)  ) {

              p_dOdometryData = compute_odometry_data();
              p_dEncPos = get_encoder_positions();
              if( fabs(dStopPosLeft - p_dEncPos[0]) <= MIN_DIST ) { set_motor_speed(-MIN_SPEED, MIN_SPEED); }
      }
    }
  }
  // the given angle is reached, so stop the robot ...
  stop_robot();
  
  // compute the odometry data ...
  p_dOdometryData = compute_odometry_data();
  // request the simulator to perform a simulation step [ms] ...
  wb_robot_step(TIME_STEP); 
}

void set_motor_speed(double dSpeedL, double dSpeedR) {
  // actulalize the global speed variables ...
  dMSpeed[0] = dSpeedL;
  dMSpeed[1] = dSpeedR;
  // set the speed values to both motors ...
  wb_differential_wheels_set_speed(dMSpeed[0], dMSpeed[1]);
}

double* get_encoder_positions() {
  static double dEncPos[2];
  
  // read the encoder positions values of the wheels ...
  dEncPos[0] = wb_differential_wheels_get_left_encoder();
  dEncPos[1] = wb_differential_wheels_get_right_encoder();
  //if(iSim == 1) {
    wb_robot_step(TIME_STEP); // performing a simulation step in [ms] ...
  //}
  
  return dEncPos;
}

double* compute_odometry_data() {
  static double dOdometryData[3];
  double *p_dEncPos; 
  
  double dNTicksLeft = 0.0f;
  double dNTicksRight = 0.0f;
  double dLeftDist = 0.0f;
  double dRightDist = 0.0f;
  double dDispCR = 0.0f;
  static double dTheta = 0.0f;
  static double dPosX = 0.0f;
  static double dPosY = 0.0f;
  
  // get the current encoder positions of both wheels ...
  p_dEncPos = get_encoder_positions();

  // calculate the linear distance (number of ticks across a
  // given span of time (sample)) of each wheel has traveled ...
  dNTicksLeft = p_dEncPos[0] - dPrevEncPos[0];
  dNTicksRight = p_dEncPos[1] - dPrevEncPos[1];
  // convert the number of the passed ticks (covered by each wheel) to meters ...
  dLeftDist = dNTicksLeft / ENCODER_RESOLUTION * WHEEL_RADIUS;
  dRightDist = dNTicksRight / ENCODER_RESOLUTION * WHEEL_RADIUS;

  // calculate the displacement (distance) from the center C
  // of the robot during the time span (sample) ...
  dDispCR = (dLeftDist + dRightDist)/2.0f;
  // calculate the orientation (rotation) theta ...
  dTheta += (dLeftDist - dRightDist)/(WHEELBASE);
  
  // convert/clip the orientation in degrees (+/-) ...
  //dTheta -= (double)((int)(dTheta/2.0f*M_PI))*(2.0f*M_PI); // ??
  
  // calculate the cartesian cooridnates (x, y) of the robot ...
  dPosX += dDispCR * sin(dTheta);
  dPosY += dDispCR * cos(dTheta);

  // update the previous encoder positions ...
  dPrevEncPos[0] = p_dEncPos[0]; //dEncPosLeft;
  dPrevEncPos[1] = p_dEncPos[1]; //dEncPosRight;
  
  // write the calculated values into the buffer and return ...
  dOdometryData[0] = dPosX; 
  dOdometryData[1] = dPosY;
  dOdometryData[2] = dTheta;
  
  return dOdometryData;
}

void UMBmark(double dSpeed, double dDistance) {
  // activate some LEDs to show that 
  // the calibration process has started ...
  set_leds(1);
  // measure and the calibrate the values of the odometry:
  measure_cw(dSpeed, dDistance); // cw direction ...
  
  // do not reset the wheel encoders between the movings!
  // only at the begin (bevore moving - initialization),
  // else the robot starting to spinning round!
  //wb_differential_wheels_set_encoders(0.0f, 0.0f);
  
  measure_ccw(dSpeed, dDistance); // ccw direction ...
  // finally stop the robot (precation) ...
  stop_robot();
}

void measure_cw(double dSpeed, double dDistance) {
  int i,j;
  
  // cw measurements ...
  for (i=0; i < NTOURNAMENTS; i++) {
    // compute the current odometry data ...
    p_dOdometryData = compute_odometry_data();
    // moving a square cw ...
    for(j=0; j < 4; j++) {
      move_forward(dSpeed, dDistance);
      turn_right(dSpeed);
    }
    //move_forward(dSpeed, dDistance);
    //turn_right(dSpeed);
    //move_forward(dSpeed, dDistance);
    //turn_right(dSpeed);
    //move_forward(dSpeed, dDistance);
    //turn_right(dSpeed);
    //move_forward(dSpeed, dDistance);

    // performing a simulation step ...
    wb_robot_step(TIME_STEP);
  }
  
  // actualize the odometry values (precaution) ...
  p_dOdometryData = compute_odometry_data();
}

void measure_ccw(double dSpeed, double dDistance) {
  int i,j;
  
  // compute the odometry data ...
  p_dOdometryData = compute_odometry_data();
  // turn the robot right for moving the same square ccw ...
  turn_right(dSpeed);
  
  // ccw measurements ...
  for (i=0; i < NTOURNAMENTS; i++) {
    // get the current odometry values ...
    p_dOdometryData = compute_odometry_data();

    // moving a square ccw ...
    for(j=0; j < 4; j++) {
      move_forward(dSpeed, dDistance);
      turn_left(dSpeed);
    }

    wb_robot_step(TIME_STEP);
  }
  
  // actualize the odometry values (precaution) ...
  p_dOdometryData = compute_odometry_data();
}

void set_leds(int iActive) {
  int i;
  for(i=0; i < 3; i++) {
    wb_led_set(led[i], iActive);
  }
}