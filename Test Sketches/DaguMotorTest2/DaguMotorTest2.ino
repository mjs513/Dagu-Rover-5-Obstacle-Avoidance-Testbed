#include <Encoder.h>
#include <Wire.h>
#include <LSM303.h>
#include <Yamartino.h>
#include <Streaming.h>
#include "Constants.h"
/*
Created by: William Garrido - FriedCircuits.us
Exmaple of Dagu 4 Channel Motor Controller with 4 motors on the Rover 5
Encoder support is provided by Teensy Encoder Library.

telem processing code is borrowed from
https://github.com/hbrobotics/ros_arduino_bridge

*/

Yamartino yamartino(compass_avg_cnt);
LSM303 compass;
float roll, pitch;
float fXg = 0;
float fYg = 0;
float fZg = 0;
int roam = 0;

#include "IOpins.h"
#include <elapsedMillis.h>
//telem input
#define telem Serial
//#define telem Serial3 // bluetooth

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

//Character arrays to hold the first argument
char argv1[16];
char argv2[16];

// A pair of varibles to help parse telem commands (thanks Fergs)
int arg = 0;
int index_ = 0;

// The arguments converted to integers
long arg1;
long arg2;

bool boolMove = false;
const byte CW = 0;
const byte CCW = 1;

unsigned time = millis();

long oldPosition  = -999;
//Set Motor Speed
const int speed = 50;
const int turnSpeed = 150; 
int turn_time_mult = 2;
int turn_time;

elapsedMillis motorFwd;
elapsedMillis motorFwdRunTime;
elapsedMillis motorTurnTime;
elapsedMillis motorRevTime;
elapsedMillis turn_timer;



void resetCommand() { 
   cmd = NULL; 
   memset(argv1, 0, sizeof(argv1)); 
   memset(argv2, 0, sizeof(argv2)); 
   arg1 = 0; 
   arg2 = 0; 
   arg = 0; 
   index_ = 0; 
 } 

void setup(){
  telem.begin(57600);
  telem.println("Rover 5 Example Sketch");

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

	pinMode(dir_lf, OUTPUT);
	pinMode(CURRENTLF, INPUT);
	pinMode(dir_rr, OUTPUT); 
	pinMode(CURRENTRR, INPUT);
	pinMode(dir_lr, OUTPUT); 
	pinMode(CURRENTLR, INPUT);
	pinMode(dir_rf, OUTPUT); 
	pinMode(CURRENTRF, INPUT);

}

void loop(){
  while (telem.available() > 0) {
    int val = telem.read();  //read telem input commands

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
      if (boolMove){
    if(millis() > (time+500)){
      //telem.print("LF-Current: ");
      //telem.println(analogRead(CURRENTLF));
      
      //telem.print("RR-Current: ");
      //telem.println(analogRead(CURRENTRR));
      
      //telem.print("LR-Current: ");
      //telem.println(analogRead(CURRENTLR));
      
      //telem.print("RF-Current: ");
      //telem.println(analogRead(CURRENTRR));

      //compass_update();
    
    getTicks_noreset();
        
      time=millis();
    }
  }
}

        //Replace with Run once or other lib.
        //currentTime = millis();
        //if(motorFwd > 200){
          //Cycle through obstacle avoidance sensors for emergency stop
          //read_sensors();
          //oneSensorCycle();
          //if(obs_array[3] == 1 || obs_array[0] == 1 || obs_array[1] == 1 || obs_array[2] == 1) {
          //  mStop();
          //  break; 
          //}
          //motorFwd = 0;
        //}
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
      mReverse();
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
    //goRoam();
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


  


