//============================================================================
//    Sketch to test various technicques in robotic car design such as
//    obstacle detection and avoidance, compass as turn guide,
//    motor control, etc.
//    Copyright (C) 2015  Michael J Smorto
//    https://github.com/mjs513/Sainsmart_4WD_Obstacle_Avoid_TestBed.git
//    FreeIMU@gmail.com
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with this program; if not, write to the Free Software Foundation, Inc.,
//    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

#define telem Serial3
//#define telem Serial3 // bluetooth

// The serial connection to the GPS device
#define ss Serial2

//Enable 400Khz I2C bus speed
const boolean fastmode = true;

//GPS Baud Rate
#define GPSBaud 57600

//int fowardheadThreshold = 52;  	//30.48,41,51,52      headservo - obs[3]
//int lcThreshold = 50; 	        //40.64,38,45,50      sonarlc obs[1]
//int lcIRthreshold = 40;  //was 45
//int sideSensorThreshold = 45;	        //50.8,38,45,41,45,36	sonarll (points to right) obs[0]
//
//const int obsDist = 27;
int obsDist = 47; //was 47
int sidedistancelimit = 37;  // was 27
int fowardheadThreshold = 37; //was 49, 39, 29; was 27
int lcThreshold = 36;         // was 47,27; was 26
int lcIRthreshold = 35;  //was 45, last 47; was 27
int sideSensorThreshold = 37; //was 42; was 27
int max_IR_distance = 200;
int turnCounter = 0;

const int backupSensorThreshold = 17;		//17.78 - not implemented yet
const int backup_high = 500;
const int backup_low = 100;

//Set Motor Speed
const int speed = 50;
const int turnSpeed = 150;
const float lf_mtr_adj = 1.5;
const float rr_mtr_adj = 0.97;
const float lr_mtr_adj = 1.15;
const float rf_mtr_adj = 1.0;
int turn_time_mult = 2;
int turn_time;

// Ground surface type, default hard surface
int gnd_type = 1;

#define v2Amps 0.004f

const int left_37 = 250;	//was 387 for 37 deg, 250 for 22deg
const int right_37 = 295;	//was 440 for 37, 295 for 22 deg
const int left_57 = 461;	//was 571 for 57, 461 for 45
const int right_57 = 519;	//was 636 for 57, 519 for 45
                                          
#define M_PI 3.14159265359

//compass reads
const int compass_avg_cnt = 20;
const float alpha = 0.5;

//sets up servo angles
const int head_fwd = 90; 
const int head_lft = 0;
const int head_rt = 180;
const int head_ldiag = 45;
const int head_rdiag = 135;

#define SONAR_NUM     4    			// Number or sensors.
#define MAX_DISTANCE 100   			// Maximum distance (in cm) to ping. was 200
#define PING_INTERVAL 32   			// Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
                                          // was set to 33; was 40
// the interval in mS 
//#define interval 7500    //was 7500
#define interval 100
#define interval1 2000
#define defaultTurnTime 1000
#define defaultFwdTime 7000
#define defaultRevTime 700

//Bubble Rebound Parameters
const float V = 21;
const float Ki = 0.2;

const int N = 25;  //was 10, 12 for 12 readings, was 12
const int angle = 7.5;  //was 20 degrees, was 15 for 12

//rcarduino constants
#define RC_NEUTRAL_STEERING 1506  //1504 was 1506
#define RC_NEUTRAL_THROTTLE 1511  //1500 was 1511

#define RC_MAX_STEERING 1996
#define RC_MAX_THROTTLE 2000

#define RC_MIN_STEERING 1016  //1012 was 1016
#define RC_MIN_THROTTLE 1014  //1012 was 1014

#define RC_DEADBAND 50

uint16_t unSteeringMin = RC_MIN_THROTTLE;
uint16_t unSteeringMax = RC_MAX_STEERING;
uint16_t unSteeringCenter = RC_NEUTRAL_STEERING;

uint16_t unThrottleMin = RC_MIN_THROTTLE;
uint16_t unThrottleMax = RC_MAX_THROTTLE;
uint16_t unThrottleCenter = RC_NEUTRAL_THROTTLE;

		  
uint16_t ThrottleDeadBandMax = RC_NEUTRAL_THROTTLE + RC_DEADBAND;
uint16_t ThrottleDeadBandMin = RC_NEUTRAL_THROTTLE - RC_DEADBAND;
uint16_t SteeringDeadBandMax = RC_NEUTRAL_STEERING + RC_DEADBAND;
uint16_t SteeringDeadBandMin = RC_NEUTRAL_STEERING - RC_DEADBAND;

#define PWM_MIN 0
#define PWM_MAX 255

#define GEAR_NONE 0
#define GEAR_IDLE 1
#define GEAR_FULL 2
#define GEAR_NEUTRAL 3

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_RIGHT 3
#define DIRECTION_ROTATE_LEFT 4

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

#define IDLE_MAX 90

#define MODE_RUN 1
#define MODE_PROGRAM 0

uint8_t gMode = MODE_RUN;
uint32_t ulProgramModeExitTime = 0; 







