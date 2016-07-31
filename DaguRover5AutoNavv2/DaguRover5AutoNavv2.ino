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
//
// ---------------------------------------------------------------------------
// Code based on example code from following:
// Dagu Rover 5 Motor Control:
//	  Created by: William Garrido - FriedCircuits.us
//	  Exmaple of Dagu 4 Channel Motor Controller with 4 motors on the Rover 5
//    Encoder support is provided by Teensy Encoder Library.
//    telem processing code is borrowed from
//    https://github.com/hbrobotics/ros_arduino_bridge
// Sainsmart Obstacle Avoidance Robot:
//    http://www.mkme.org/index.php/arduino-sainsmart-4wd-robot/
//    https://github.com/gmossy/Sainsmart-4WD-Robot
// Wheel Encoders - the DAGU Simple Encoder Kit
//    http://www.bajdi.com/adding-encoders-to-those-cheap-yellow-motors/
//    http://letsmakerobots.com/node/38636
//    http://playground.arduino.cc/Main/ReadingRPM
//    http://elimelecsarduinoprojects.blogspot.com/2013/06/measure-rpms-arduino.html 
//  Compass Averaging
//    Yamartino Library, Christopher Baker  https://github.com/SAIC-ATS/Algorithms.git
//  Obstacle avoidance approaches
//    http://homepages.engineering.auckland.ac.nz/~pxu012/mechatronics2013/group7/index.html
//    http://homepages.engineering.auckland.ac.nz/~pxu012/mechatronics2013/group7/software.html
//  IR Sensing
//    http://letsmakerobots.com/node/40502
//    http://adhocnode.com/arduino-irrecv-module/
//    http://arduino-info.wikispaces.com/IR-RemoteControl
//
//  More to follow as I add PID controls and Bubble rebound algorithm for obstacle
//  avoidance
//
//  Advance obstacle avoidance algoritm:
//    Susnea, I., Viorel Minzu, Grigore Vasiliu. Simple, real-time obstacle
//    avoidance algorithm for mobile robots. in 8th WSEAS International
//    Conference on Computational intelligence, man-machine systems and
//    cybernetics (CIMMACS'09) 2009
//    http://www.wseas.us/e-library/conferences/2009/tenerife/CIMMACS/CIMMACS-03.pdf
//
//    Ulrich, I., and Borenstein, J., "VFH+: Reliable Obstacle
//    Avoidance for Fast Mobile Robots", IEEE Int. Conf. on Robotics and Automation, May 1998,
//    pp. 1572-1577. http://www.cs.cmu.edu/~iwan/papers/vfh+.pdf
//
//    VFH Gap routine extracted from Orca: Components for Robotics, vfh algorithm
//    http://orca-robotics.sourceforge.net/
//
//  Use of angle sensors and draft avoidance logic references
//    Obstacle Avoidance in the Real World: 
//      http://www.pirobot.org/blog/0003/
//      https://youtu.be/4yXysCkcdwQ
//       Where Am I? Place Recognition Using Omni-directional Images and Color Histograms
//       http://forums.trossenrobotics.com/tutorials/introduction-129/where-am-i-place-recognition-using-omni-directional-images-and-color-histograms-3253/
//    Megamouse An Autonomous Maze Solving Robot
//       https://djgeorgevas.com/static/megamouse.pdf
//
//============================================================================================
// rcarduino.blogspot.com
//
// A simple approach for reading two RC Channels from a hobby quality receiver
// and outputting to the common motor driver IC the L293D to drive a tracked vehicle
//
// We will use the Arduino to mix the channels to give car like steering using a standard two stick
// or pistol grip transmitter. The Aux channel will be used to switch and optional momentum mode on and off
//
// See related posts -
//
// Reading an RC Receiver - What does this signal look like and how do we read it - 
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
//
// The Arduino library only supports two interrupts, the Arduino pinChangeInt Library supports more than 20 - 
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
//
// The Arduino Servo Library supports upto 12 Servos on a single Arduino, read all about it here - 
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// The wrong and then the right way to connect servos to Arduino
// http://rcarduino.blogspot.com/2012/04/servo-problems-with-arduino-part-1.html
// http://rcarduino.blogspot.com/2012/04/servo-problems-part-2-demonstration.html
//
// Using pinChangeInt library and Servo library to read three RC Channels and drive 3 RC outputs (mix of Servos and ESCs)
// http://rcarduino.blogspot.com/2012/04/how-to-read-multiple-rc-channels-draft.html
//
// rcarduino.blogspot.com
//
// -------------------------------------------------------------------

#include <StandardCplusplus.h>
#include <vector>


#include <NewPing.h>
#include <Servo.h>         //servo library
#include <Encoder.h>
#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Streaming.h>
#include <elapsedMillis.h>
#include <TinyGPS++.h>
#include <PinChangeInt.h>

#include "Constants.h"
#include "IOpins.h"

Servo headservo;
// The TinyGPS++ object
TinyGPSPlus gps;

//Custom GPS Strings
TinyGPSCustom UTC(gps, "GNRMC", 1); 
TinyGPSCustom Status(gps, "GNRMC", 2); 
TinyGPSCustom Long(gps, "GNRMC", 5); 
TinyGPSCustom EW(gps, "GNRMC", 6); 
TinyGPSCustom Lat(gps, "GNRMC", 3);
TinyGPSCustom SOG(gps, "GNRMC", 7);
TinyGPSCustom COG(gps, "GNRMC",8);

//Globals
float roll, pitch;
float fXg = 0;
float fYg = 0;
float fZg = 0;

// Set elapsed time constants
elapsedMillis motorFwd;
elapsedMillis motorFwdRunTime;
elapsedMillis motorTurnTime;
elapsedMillis motorRevTime;
elapsedMillis turn_timer;

// the interval in mS 
unsigned long currentTime;
unsigned long currentTime_avg;
unsigned long sensorRunTime;

// set sonar sensor configuration
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

uint8_t obs_array[5];
unsigned int cm_head[5];

// sets up IR sensor variables
int frtIRdistance, rearIRdistance;

// global for heading from compass
float yar_heading;

/* Set the delay between fresh samples for BNO055*/
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// sets globals for distance and turn directions
//int Speed;
//int turn_time_mult;
boolean clockwise;
const byte CW = 0;
const byte CCW = 1;
int minDistance, nextTurn;
String nextMove, lastMove, turnDirection;
int minIndex;
float mina;

int roam = 0;	// toggle variable for roaming mode
int rc_mode_toggle = 0;
int rc_sw_on = 0;

//encoder variables
int ticksLF;
int ticksRF;
int ticksRR;
int ticksLR;

unsigned time = millis();
float fit_time, rebound_angle;
int delay_time;

long oldPosition  = -999;

//rcarduino shared variables
// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unRCInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;
uint32_t ulRCStart;

uint8_t throttleLeft;
uint8_t throttleRight;
int tDeadZoneRange, sDeadZoneRange;

//

void setup(){
	telem.begin(57600);
	telem.println("Rover 5 Obstacle Avoidance");

	Wire.begin();
	ss.begin(GPSBaud);  //GPS Device Baud Rate
    
	//==================================================
	// Change I2C bus speed from 100KHz to 400KHz
	//==================================================
    
	#if defined(__AVR_ATmega128) || defined( __AVR_ATmega2560__ ) // only valid on AVR, not on 32bit platforms (eg: Arduino 2, Teensy 3.0)
      if(fastmode) { // switch to 400KHz I2C - eheheh
        TWBR = ((F_CPU / 400000L) - 16) / 2; // see twi_init in Wire/utility/twi.c
      }
    #elif defined(__arm__)
      if(fastmode) {
        #if defined(CORE_TEENSY) && F_BUS == 48000000
          I2C0_F = 0x1A;  // Teensy 3.0 at 48 or 96 MHz
          I2C0_FLT = 2;
        #elif defined(CORE_TEENSY) && F_BUS == 24000000
          I2C0_F = 0x45;  // Teensy 3.0 at 24 MHz
          I2C0_FLT = 1;
        #endif
      }
    #elif defined(__SAM3X8E__) //Arduino Due
      if(fastmode) { // switch to 400KHz I2C - eheheh
        TWBR = ((F_CPU / 400000L) - 16) / 2; 
      }
	#endif
    
	//==================================================
	// Initial AltIMU10 v3
	//==================================================
    
	BNO055_Init();
	delay(100);
 
	telem.println();
	telem.println("Ready to receive telem Commands![g, f, b, r, l, s, t, c]"); // Tell us I"m ready

    //telem.println("My Commands are: ");
    //telem.println("f:forward");
    //telem.println("b:backward");
    //telem.println("r:right");
    //telem.println("l:left");
    //telem.println("s:stop");
    //telem.println("g:ground type, g1 = hard, g2 = carpet");
    //telem.println("t:toggleRoam");

    //signal output port
    //set all of the outputs for the motor driver
    pinMode(pwm_lf, OUTPUT);
	pinMode(dir_lf, OUTPUT);
	pinMode(CURRENTLF, INPUT);
	pinMode(pwm_rr, OUTPUT);
	pinMode(dir_rr, OUTPUT); 
	pinMode(CURRENTRR, INPUT);
	pinMode(pwm_lr, OUTPUT);
	pinMode(dir_lr, OUTPUT); 
	pinMode(CURRENTLR, INPUT);
	pinMode(pwm_rf, OUTPUT);
	pinMode(dir_rf, OUTPUT); 
	pinMode(CURRENTRF, INPUT);
	
	// headservo interface
	headservo.attach(HeadServopin);
	headservo.write(head_fwd);
	delay(100);

 	//intialize rc channel internal interupts
	// ISR for all channels

	PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE);
	PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering, CHANGE);
  PCintPort::attachInterrupt(RCMODE_IN_PIN, togRCMode, CHANGE);
  
  throttleLeft = throttleRight = speed;

}

void loop(){
  while (telem.available() > 0) {
    int val = telem.read();	//read telem input commands

    turn_time_mult = telem.parseInt();
    if(turn_time_mult == 0)
                turn_time_mult = 4;          

    switch(val)
    {
    case 'g':
		if(turn_time_mult == 2) {
			obsDist = 47; //was 47
			sidedistancelimit = 27;  // was 27
			fowardheadThreshold = 27; //was 49, 39, 29; was 27
			lcThreshold = 26;         // was 47,27; was 27
			lcIRthreshold = 25;  //was 45, last 47; was 27
			sideSensorThreshold = 27; //was 42; was 27
			gnd_type = 2;  // carpet
		}
		break;
		
      case 'f' : 
        motorFwdRunTime = 0;
        motorFwd = 0;
        
        telem.println("Rolling Forward!");
        // zeros out encoder counts and reads encoders zero value
        encA.write(0); encB.write(0); encC.write(0); encD.write(0);
        getTicks_noreset();
        mForward();
        
       while(motorFwdRunTime < defaultFwdTime){
        //currentTime = millis();
        if(motorFwd > 200){
          //Cycle through obstacle avoidance sensors for emergency stop
          read_sensors();
          oneSensorCycle();
          if(obs_array[3] == 1 || obs_array[0] == 1 || obs_array[1] == 1 || obs_array[2] == 1) {
            mStop();
            break; 
          }
          motorFwd = 0;
        }
       }
      mStop(); 
      getTicks_noreset();
      
      motorFwdRunTime = 0;
      break;
      
    case 'l' :
      motorTurnTime = 0;
      //===================================================
      // Used for turn calibration curve
      //===================================================
      //compass_update();
      //telem << "Current heading: " << yar_heading << endl;
      //telem << "Turn Multiplier: " << turn_time_mult << endl;
      telem.println("Turning Left!");
      mLeft();
      //delay(400);  //was 2000
      delay(turn_time_mult * 100);
      mStop();
      //compass_update();
      //telem << "Changed heading: " << yar_heading << endl;
      while(motorTurnTime < defaultTurnTime) {
       }
      mStop();
      motorTurnTime = 0;
      break;

    case 'r' :
      //===================================================
      // Used for turn calibration curve
      //===================================================
      //telem << "Turn Multiplier: " << turn_time_mult << endl;
      //telem.println("Turning Right!");
      //compass_update();
      //telem << "Current heading: " << yar_heading << endl;
      mRight();
      //delay(400);
      delay(turn_time_mult * 100);
      mStop();
      //compass_update();
      //telem << "Changed heading: " << yar_heading << endl;
      while(motorTurnTime < defaultTurnTime) {
        }
      mStop();
      break;
             
    case 'b' :
      motorRevTime = 0;    
      telem.println("Moving Backward!");
      //moveBackward(motorSpeed);
      mBackward();
      while(motorRevTime < defaultRevTime) {
        }
      mStop();
      motorRevTime = 0;
      break;
      
    case 's' :      
      telem.println("Stop!");
      mStop();
      break;
      
    case 't' :      
      telem.println("toggle Roam Mode"); 
      toggleRoam();
      break;
      
    case 'c' :
      telem.println("toggle RC control mode");
      toggleRC();
      break;
    }      
    delay(1);  
    telem.println("I'm Ready to receive telem Commands![g, f, b, r, l, s, t, c]"); // Tell us I"m ready
  }
      
  if(roam == 0){ 
      //just listen for telem commands and wait
      }
  else if(roam == 1){  //If roam active- drive autonomously
    goRoam();
    }

  if(rc_mode_toggle == 0){ 
      //just listen for telem commands and wait
      }
  else if(rc_mode_toggle == 1) {  //If roam active- drive autonomously
    goRC();
    }
    
  if(unRCInShared > RC_MODE_TOGGLE && rc_mode_toggle == 0) {
        telem << "toggle RC Mode On via SW" << endl; 
        rc_sw_on = 1;
        toggleRC();
      }
}

void toggleRoam(){
  // This method chooses to make the robot roam or else use the telem command input.
  if(roam == 0){
   roam = 1;
   telem.println("Activated Roam Mode");
  } else {
    roam = 0;
    mStop();
    telem.println("De-activated Roam Mode");
    telem.println("I'm Ready to receive telem Commands![g, f, b, r, l, s, t, c]"); // Tell us I"m ready

  }
}

void toggleRC(){
  // This method chooses to make the robot roam or else use the telem command input.
  if(rc_mode_toggle == 0){
   rc_mode_toggle = 1;
   telem.println("Activated RC Mode");
  } else {
    rc_mode_toggle = 0;
    throttleLeft = throttleRight = speed;
    mStop();
    telem.println("De-activated RC Mode");
	  telem.println("I'm Ready to receive telem Commands![g, f, b, r, l, s, t, c]"); // Tell us I"m ready
  }
}

void goRoam() {  
   read_sensors();   
   oneSensorCycle(); 
   decide_direction();
    
}

void goRC() {  
   rc_control();   
 
}

//**********************************************************
//IsTime() function - David Fowler, AKA uCHobby, http://www.uchobby.com 01/21/2012

#define TIMECTL_MAXTICKS  4294967295L
#define TIMECTL_INIT      0

int IsTime(unsigned long *timeMark, unsigned long timeInterval){
  unsigned long timeCurrent;
  unsigned long timeElapsed;
  int result=false;
  
  timeCurrent=millis();
  if(timeCurrent<*timeMark) {  //Rollover detected
    timeElapsed=(TIMECTL_MAXTICKS-*timeMark)+timeCurrent;  //elapsed=all the ticks to overflow + all the ticks since overflow
  }
  else {
    timeElapsed=timeCurrent-*timeMark;  
  }

  if(timeElapsed>=timeInterval) {
    *timeMark=timeCurrent;
    result=true;
  }
  return(result);  
}


