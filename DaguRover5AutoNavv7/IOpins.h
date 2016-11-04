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

//Dagu 4 channel motor controller
//Motors
const int pwm_lf = 4;  //PWM for CH1 LF
const int pwm_rr = 7;  //PWM for CH4 RR  - Revserved Encoder pins to make postive
const int pwm_lr = 9;  //PWM for CH2 LR
const int pwm_rf = 10;  //PWM for CH3 RF

const int dir_lf = 45;  //DIR for CH1
const int dir_rr = 42;  //DIR for CH4
const int dir_lr = 44;  //DIR for CH2
const int dir_rf = 43;  //DIR for CH3

//Current Sensors
const int CURRENTLF = A12; 
const int CURRENTRR = A13; 
const int CURRENTLR = A14; 
const int CURRENTRF = A15; 
const int CURRENT_LIMIT = (1024 / 5) * 2.6;  // amps

//Encoder Interrupts
//Needs Teensy Encoder library
const int encA1 = 2;
const int encA2 = 46;
const int encB1 = 3;
const int encB2 = 47;
const int encC1 = 18;
const int encC2 = 48;
const int encD1 = 19;
const int encD2 = 49;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encA(encA1, encA2);  // lf
Encoder encB(encB1, encB2);  // rf
Encoder encC(encC1, encC2);  // rr
Encoder encD(encD1, encD2);  // lr

//////////////////////////////////////////////////////
//RC Control, setup interupts


const int HeadServopin = 11; // signal input of headservo, was 8.
//const int head_tilt_pin = 3; // signal input for headservo tilt

NewPing sonarll(34, 35, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarlc(32, 33, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarlr(28, 29, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarhd(39, 38, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


//IR Sensor Pins
const int leftIRsensor = A3;
const int rightIRsensor = A2;

// Assign your rc channel in pins
int THROTTLE_IN_PIN = 51;
int STEERING_IN_PIN = 50;
int RCMODE_IN_PIN = 52;










