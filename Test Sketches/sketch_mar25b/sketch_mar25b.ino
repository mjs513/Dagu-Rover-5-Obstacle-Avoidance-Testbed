
// Dagu 5 Chassis example.
// Author: Nick Gammon
// Date:   20th January 2012


#include <Wire.h>

const unsigned long TIME_BEFORE_WE_GIVE_UP = 1000;  // ms
const int WIRELESS_BAUD_RATE = 600;

const byte FWD = 0;
const byte BACKWD = 1;

const float LOW_BATTERY_VOLTAGE = 7.5;

// analog pins

const byte CURRENTA = 0;  // A0
const byte CURRENTB = 1;  // A1
const byte BATTERY_CHECK = 2; // A2

// digital pins

const byte  LEFT_DIRECTION = 45;
const byte  LEFT_MOTOR = 4;

const byte  RIGHT_DIRECTION = 44;
const byte  RIGHT_MOTOR = 10;

const byte RED_LED = 13;

#define CURRENT_LIMIT (1024 / 5) * 2.6  // amps

#define TIME_FORWARDS 10000
#define TIME_BACKWARDS 10000
#define TIME_TURN 1000

#define FULL_SPEED 255
#define SPEED_ADJUST 5

void jog (const byte direction)
{
  digitalWrite (LEFT_DIRECTION, direction);  
  digitalWrite (RIGHT_DIRECTION, direction); 

  analogWrite (LEFT_MOTOR, 128);
  analogWrite (RIGHT_MOTOR, 128);
  delay (1250);
  analogWrite (LEFT_MOTOR, 0);
  analogWrite (RIGHT_MOTOR, 0);
  delay (1250);

}  // end of jog


// callback routines

void fWrite (const byte what)
{
  Serial.print (what);  
}  // end of fWrite

int fAvailable ()
{
  return Serial.available ();  
}  // end of fAvailable

int fRead ()
{
  return Serial.read ();  
}  // end of fRead

void setup ()
{
  analogWrite (LEFT_MOTOR, 0);
  analogWrite (RIGHT_MOTOR, 0);

  pinMode (LEFT_DIRECTION, OUTPUT);
  pinMode (RIGHT_DIRECTION, OUTPUT);

  Serial.begin (WIRELESS_BAUD_RATE);

  // jog wheels to show we are ready ...

  jog (FWD);
  jog (BACKWD);

  // low battery warning LED  
  pinMode (RED_LED, OUTPUT);

}  // end of setup

// ---------------- MAIN LOOP -------------------

unsigned long lastCommand;

void loop ()
{

  jog (FWD);
  jog (BACKWD); 

  // fiddle around making turning better


  // set motor speed, no less than 0 and no greater than 25
}  // end of loop

