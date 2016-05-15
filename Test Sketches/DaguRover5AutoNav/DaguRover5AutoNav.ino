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
// -------------------------------------------------------------------

#include <StandardCplusplus.h>
#include <vector>


#include <NewPing.h>
#include <Servo.h>         //servo library
#include <Encoder.h>
#include <Wire.h>
#include <LSM303.h>
#include <Yamartino.h>
#include <Streaming.h>
#include <elapsedMillis.h>

#include "Constants.h"
#include "IOpins.h"

Yamartino yamartino(compass_avg_cnt);
LSM303 compass;
float roll, pitch;
float fXg = 0;
float fYg = 0;
float fZg = 0;

Servo headservo;
elapsedMillis motorFwd;
elapsedMillis motorFwdRunTime;
elapsedMillis motorTurnTime;
elapsedMillis motorRevTime;

// the interval in mS 
unsigned long currentTime;
unsigned long currentTime_avg;
unsigned long sensorRunTime;


unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

uint8_t obs_array[5];
unsigned int cm_head[5];

int frtIRdistance, rearIRdistance;
int leftCounter, rightCounter;

float yar_heading;


int minDistance, nextTurn;
String nextMove, lastMove, turnDirection;
int minIndex;
float mina;

int roam = 0;

//int Speed;
//int turn_time_mult;
boolean clockwise;

const byte CW = 0;
const byte CCW = 1;

unsigned time = millis();

long oldPosition  = -999;

void setup(){
	telem.begin(57600);
	telem.println("Rover 5 Obstacle Avoidance");

	Wire.begin();
    
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
    
	AltIMU_Init();
	delay(100);
 
	telem.println();
	telem.println("Ready to receive telem Commands![f, b, r, l, s, t]"); // Tell us I"m ready

    //telem.println("My Commands are: ");
    //telem.println("f:forward");
    //telem.println("b:backward");
    //telem.println("r:right");
    //telem.println("l:left");
    //telem.println("s:stop");
    //telem.println("t:toggleRoam");

    //signal output port
    //set all of the outputs for the motor driver
    
	pinMode(dir_lf, OUTPUT);
	pinMode(CURRENTLF, INPUT);
	pinMode(dir_rr, OUTPUT); 
	pinMode(CURRENTRR, INPUT);
	pinMode(dir_lr, OUTPUT); 
	pinMode(CURRENTLR, INPUT);
	pinMode(dir_rf, OUTPUT); 
	pinMode(CURRENTRF, INPUT);
	
	// headservo interface
	headservo.attach(HeadServopin);
	headservo.write(head_fwd);
	delay(100);

}

void loop(){
  while (telem.available() > 0) {
    int val = telem.read();	//read telem input commands

    turn_time_mult = telem.parseInt();
    if(turn_time_mult == 0)
                turn_time_mult = 4;          
    
    switch(val)
    {
      case 'f' : 
        motorFwdRunTime = 0;
        motorFwd = 0;
        
		telem.println("Rolling Forward!");
		mForward();
        
		//Read Encoders and current sense
		// TBD
		//
		//////////////////////////////////
       while(motorFwdRunTime < defaultFwdTime){
    
        //Replace with Run once or other lib.
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
    }      
    delay(1);  
    telem.println("I'm Ready to receive telem Commands![f, b, r, l, s, t]"); // Tell us I"m ready
  }
      
  if(roam == 0){ 
      //just listen for telem commands and wait
      }
  else if(roam == 1){  //If roam active- drive autonomously
    goRoam();
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
  }
}

void goRoam() {  
   read_sensors();   
   oneSensorCycle(); 
   decide_direction();
    
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






