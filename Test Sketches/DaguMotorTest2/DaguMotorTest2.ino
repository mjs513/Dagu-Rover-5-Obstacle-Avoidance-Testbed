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


#include "IOpins.h"

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

void runCommand(){
  
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);
    
    switch (cmd){
    
      case 'f':
            telem.print("Forward: ");  
            telem.println(arg1);   
            mForward(arg1);
      break;
      
      case 'l':
          telem.print("Left: "); 
          telem.println(arg1);  
          mLeft(arg1); 
      break;
      
      case 'r':
         telem.print("Right: ");
         telem.println(arg1);
         mRight(arg1);     
      break;
      
      case 'b':
        telem.print("Reverse: "); 
        telem.println(arg1); 
        mReverse(arg1);      
      break;
      
      case 's':
        mStop();
        telem.println("Stopped");
      break;
  }
  
  resetCommand();

}

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
    
    // Read the next character
    chr = telem.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index_] = NULL;
      else if (arg == 2) argv2[index_] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index_] = NULL;
        arg = 2;
        index_ = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index_] = chr;
        index_++;
      }
      else if (arg == 2) {
        argv2[index_] = chr;
        index_++;
      }
    }
  }


  
  if (boolMove){
    if(millis() > (time+500)){
      telem.print("LF-Current: ");
      telem.println(analogRead(CURRENTLF));
      
      telem.print("RR-Current: ");
      telem.println(analogRead(CURRENTRR));
      
      telem.print("LR-Current: ");
      telem.println(analogRead(CURRENTLR));
      
      telem.print("RF-Current: ");
      telem.println(analogRead(CURRENTRR));

      //compass_update();
        
      time=millis();
    }
  }
}


