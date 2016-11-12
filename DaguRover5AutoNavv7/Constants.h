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
//#define telem Serial3 // telemetry

// The serial connection to the GPS device
#define gps_port Serial2
  
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

int backupSensorThreshold = 17;		//17.78 - not implemented yet
int backup_high = 500;
int backup_low = 100;

//Set Motor Speed
int speed = 50;
int turnSpeed = 150;
float lf_mtr_adj = 1.5;
float rr_mtr_adj = 0.97;
float lr_mtr_adj = 1.15;
float rf_mtr_adj = 1.0;
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
#define rad2deg 57.2957795

//compass reads
const int compass_avg_cnt = 20;
const float alpha = 0.5;
//#define DEC_ANGLE -13.1603  // -13.1603 degrees for Flushing, NY
#define DEC_ANGLE 0

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
#define defaultFwdTime 10000 //was 7000
#define defaultRevTime 700
#define defaultTelemTime 1000
#define defaultWayPointTime 1000
#define defaultOdoTime 20

//Bubble Rebound Parameters
const float V = 21;
const float Ki = 0.2;

const int N = 25;  //was 10, 12 for 12 readings, was 12
const int angle = 7.5;  //was 20 degrees, was 15 for 12

//rcarduino constants
#define RC_NEUTRAL_STEERING 1506  //1504 was 1506
#define RC_NEUTRAL_THROTTLE 1504  //1500 was 1511

#define RC_MAX_STEERING 1996
#define RC_MAX_THROTTLE 1996

#define RC_MIN_STEERING 1016  //1012 was 1016
#define RC_MIN_THROTTLE 1012  //1012 was 1014

#define RC_DEADBAND 30

#define RC_MODE_TOGGLE 1500

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
#define PWM_MAX 200

#define GEAR_NONE 0
#define GEAR_IDLE 1
#define GEAR_FULL 2
#define GEAR_NEUTRAL 3

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define RC_FLAG 3

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_RIGHT 3
#define DIRECTION_ROTATE_LEFT 4
#define DIRECTION_PIVOT_RIGHT 5
#define DIRECTION_PIVOT_LEFT 6

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

#define IDLE_MAX 40

#define MODE_RUN 1
#define MODE_PROGRAM 0

uint8_t gMode = MODE_RUN;
uint32_t ulProgramModeExitTime = 0; 

// Waypoints  Constants
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

#define WAYPOINT_DIST_TOLERANCE  2   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 5          // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()

struct waypoints {
    long tLat;
    long tLong;
};

waypoints waypointList[NUMBER_WAYPOINTS] = {
                                            {407746650, -738146620},
                                            {407746440, -738146110},
                                            {407746160, -738146690},
                                            {407746120, -738146460},
                                            {0,0}
                                            };

/*
waypoints waypointList[NUMBER_WAYPOINTS] = {
                                            {407745182, -738161181},
                                            {407737788, -738161530},
                                            {407738377, -738171963},
                                            {407745060, -738166693},
                                            {0,0}
                                            };                                            
*/
// Steering/turning 
int turnDirections;


// Object avoidance distances (in cm)
#define SAFE_DISTANCE 177
#define TURN_DISTANCE 102
#define STOP_DISTANCE 31


// Speeds (range: 0 - 255)
int FAST_SPEED = 150;
#define NORMAL_SPEED 125
#define TURN_SPEED 125
int TURN_SPEED_DIFF = 75;
#define SLOW_SPEED 75
#define NORMAL_SPEED speed




