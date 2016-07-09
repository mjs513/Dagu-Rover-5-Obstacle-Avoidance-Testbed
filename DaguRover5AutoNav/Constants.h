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

//Enable 400Khz I2C bus speed
const boolean fastmode = true;


//int fowardheadThreshold = 52;  	//30.48,41,51,52      headservo - obs[3]
//int lcThreshold = 50; 	        //40.64,38,45,50      sonarlc obs[1]
//int lcIRthreshold = 40;  //was 45
//int sideSensorThreshold = 45;	        //50.8,38,45,41,45,36	sonarll (points to right) obs[0]
//
//const int obsDist = 27;
const int obsDist = 47; //was 47
const int sidedistancelimit = 27;  // was 27
const int fowardheadThreshold = 27; //was 49, 39, 29; was 27
const int lcThreshold = 26;         // was 47,27; was 27
const int lcIRthreshold = 25;  //was 45, last 47; was 27
const int sideSensorThreshold = 27; //was 42; was 27
const int max_IR_distance = 200;
int turnCounter = 0;

const int backupSensorThreshold = 17;		//17.78 - not implemented yet
const int backup_high = 500;
const int backup_low = 100;

//Set Motor Speed
const int speed = 50;
const int turnSpeed = 150; 
int turn_time_mult = 2;
int turn_time;

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

