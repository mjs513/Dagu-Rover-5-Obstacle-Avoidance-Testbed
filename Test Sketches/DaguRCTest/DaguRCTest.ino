#include <PinChangeInt.h>

template <typename T>
    bool IsInBounds(const T& value, const T& low, const T& high) {
    return !(value < low) && !(high < value);
}

// MultiChannel L293D
//
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

// if stopped and turn
// rotate on spot
// if crawling
// rotate on one side
// if forward or backward
// map

#define RC_NEUTRAL_STEERING 1572
#define RC_NEUTRAL_THROTTLE 1549

#define RC_MAX_STEERING 2076
#define RC_MAX_THROTTLE 2024

#define RC_MIN_STEERING 1124
#define RC_MIN_THROTTLE 1064

#define RC_DEADBAND 40

uint16_t unSteeringMin = RC_MIN_THROTTLE;
uint16_t unSteeringMax = RC_MAX_STEERING;
uint16_t unSteeringCenter = RC_NEUTRAL_STEERING;

uint16_t unThrottleMin = RC_MIN_THROTTLE;
uint16_t unThrottleMax = RC_MAX_THROTTLE;
uint16_t unThrottleCenter = RC_NEUTRAL_THROTTLE;

#define PWM_MIN 0
#define PWM_MAX 255

#define GEAR_NONE 1
#define GEAR_IDLE 1
#define GEAR_FULL 2

// Assign your channel in pins
int THROTTLE_IN_PIN = 51;
int STEERING_IN_PIN = 50;

//Dagu 4 channel motor controller
//Motors
const int pwm_lf = 4;    //PWM for CH1 LF
const int pwm_rr = 7;   //PWM for CH4 RR  - Revserved Encoder pins to make postive
const int pwm_lr = 9;   //PWM for CH2 LR
const int pwm_rf = 10;  //PWM for CH3 RF

const int dir_lf = 45;  //DIR for CH1 LF
const int dir_rr = 42;  //DIR for CH4 RR 
const int dir_lr = 44;  //DIR for CH2 LR
const int dir_rf = 43;  //DIR for CH3 RF

const float lf_mtr_adj = 1.5;
const float rr_mtr_adj = 0.97;
const float lr_mtr_adj = 1.15;
const float rf_mtr_adj = 1.0;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

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

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

const byte CW = 0;
const byte CCW = 1;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_RIGHT 3
#define DIRECTION_ROTATE_LEFT 4

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;
uint8_t throttleLeft;
uint8_t throttleRight;

#define IDLE_MAX 150

#define MODE_RUN 1
#define MODE_PROGRAM 0

uint8_t gMode = MODE_RUN;
uint32_t ulProgramModeExitTime = 0; 


void setup()
{
  Serial.begin(9600);

  Serial.println("RCChannelsTo293");
  
  //signal output port
  //set all of the outputs for the motor driver
    
  pinMode(dir_lf, OUTPUT);
  pinMode(dir_rr, OUTPUT); 
  pinMode(dir_lr, OUTPUT); 
  pinMode(dir_rf, OUTPUT); 

  // ISR for all channels

  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE);
  PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering, CHANGE);
  Serial.println("push the controls");
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  Serial.println(bUpdateFlagsShared);
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if(bUpdateFlags & THROTTLE_FLAG)
    {
		  unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & STEERING_FLAG)
    {
		  unSteeringIn = unSteeringInShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }


  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop

  // we are checking to see if the channel value has changed, this is indicated 
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.
  if(bUpdateFlags & THROTTLE_FLAG)
    {
      // A good idea would be to check the before and after value, 
      // if they are not equal we are receiving out of range signals
      // this could be an error, interference or a transmitter setting change
      // in any case its a good idea to at least flag it to the user somehow
      unThrottleIn = constrain(unThrottleIn, unThrottleMin, unThrottleMax);
      
      if(unThrottleIn < unThrottleCenter) //For my Futaba 2ch reverse is less than center
      { //Changed unThrottleMax to min
        gThrottle = map(unThrottleIn, unThrottleCenter, unThrottleMin, PWM_MIN, PWM_MAX );
        gThrottleDirection = DIRECTION_FORWARD;
      }
      else
      {
        gThrottle = map(unThrottleIn, unThrottleMax, unThrottleCenter, PWM_MIN, PWM_MAX );
        gThrottleDirection = DIRECTION_REVERSE;
      }
 
      if(gThrottle < IDLE_MAX)
      {
        gGear = GEAR_IDLE;
      }
      else
      {
        gGear = GEAR_FULL;
      }
	}
		  
	  uint8_t ThrottleDeadBandMax = RC_NEUTRAL_THROTTLE + RC_DEADBAND;
	  uint8_t ThrottleDeadBandMin = RC_NEUTRAL_THROTTLE - RC_DEADBAND;
	  uint8_t SteeringDeadBandMax = RC_NEUTRAL_STEERING + RC_DEADBAND;
	  uint8_t SteeringDeadBandMin = RC_NEUTRAL_STEERING - RC_DEADBAND;
	  if(unThrottleIn < ThrottleDeadBandMax && unThrottleIn > ThrottleDeadBandMin &&  
		unSteeringIn < SteeringDeadBandMax && SteeringDeadBandMax > SteeringDeadBandMin)
    {
      analogWrite(pwm_lf, 0);
      analogWrite(pwm_rr, 0);
      analogWrite(pwm_lr, 0);
      analogWrite(pwm_rf, 0);
     }

  if(bUpdateFlags & STEERING_FLAG)
    {
      throttleLeft = gThrottle;
      throttleRight = gThrottle;

      gDirection = gThrottleDirection;

      // see previous comments regarding trapping out of range errors
      // this is left for the user to decide how to handle and flag
      unSteeringIn = constrain(unSteeringIn, unSteeringMin, unSteeringMax);

      // if idle spin on spot
      switch(gGear)
      {
      case GEAR_IDLE:
	    // same changes for steering as for throttle
        if(unSteeringIn < (unSteeringCenter - RC_DEADBAND))
        {
          gDirection = DIRECTION_ROTATE_RIGHT;
          // use steering to set throttle
          throttleRight = throttleLeft = map(unSteeringIn, unSteeringCenter,unSteeringMin, PWM_MIN, PWM_MAX);
        }
        else if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
        {
          gDirection = DIRECTION_ROTATE_LEFT;
          // use steering to set throttle
          throttleRight = throttleLeft = map(unSteeringIn, unSteeringMax, unSteeringCenter, PWM_MAX, PWM_MIN);
        }
        break;
      // if not idle proportionally restrain inside track to turn vehicle around it
      case GEAR_FULL:
        if(unSteeringIn < (unSteeringCenter - RC_DEADBAND))
        {
          throttleRight = map(unSteeringIn,unSteeringCenter,unSteeringMin, gThrottle, PWM_MIN);
        }
        else if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
        {
          throttleLeft = map(unSteeringIn,unSteeringMax,unSteeringCenter, PWM_MIN, gThrottle);
        }
        break;
      }

    //analogWrite (pwm_lf, throttleLeft*lf_mtr_adj);
    //analogWrite (pwm_lr, throttleLeft*lr_mtr_adj);  
    //analogWrite (pwm_rr, throttleRight*rr_mtr_adj);
    //analogWrite (pwm_rf, throttleRight*rf_mtr_adj);

    }


  if((gDirection != gOldDirection) || (gGear != gOldGear))
  {
    gOldDirection = gDirection;
    gOldGear = gGear;

  analogWrite(pwm_lf, 0);
  analogWrite(pwm_rr, 0);
  analogWrite(pwm_lr, 0);
  analogWrite(pwm_rf, 0);

  switch(gDirection)
    {
    case DIRECTION_FORWARD:  
      digitalWrite(dir_lf, CW);  
      digitalWrite(dir_rr, CCW);
      digitalWrite(dir_lr, CCW);    
      digitalWrite(dir_rf, CW);
	  
	  analogWrite (pwm_lf, throttleLeft);
      analogWrite (pwm_lr, throttleLeft);  
      analogWrite (pwm_rr, throttleRight);
      analogWrite (pwm_rf, throttleRight);
      break;
    case DIRECTION_REVERSE:
      digitalWrite(dir_lf, CCW);  
      digitalWrite(dir_rr, CW);
      digitalWrite(dir_lr, CW);    
      digitalWrite(dir_rf, CCW);
	  
	  analogWrite (pwm_lf, throttleLeft);
      analogWrite (pwm_lr, throttleLeft);  
      analogWrite (pwm_rr, throttleRight);
      analogWrite (pwm_rf, throttleRight);
      break;
    case DIRECTION_ROTATE_LEFT:
      digitalWrite(dir_lf, CCW);  
      digitalWrite(dir_rr, CCW); 
      digitalWrite(dir_lr, CW);  
      digitalWrite(dir_rf, CW);
	  
	  analogWrite (pwm_lf, throttleLeft);
      analogWrite (pwm_lr, throttleLeft);  
      analogWrite (pwm_rr, throttleRight);
      analogWrite (pwm_rf, throttleRight);
      break;
    case DIRECTION_ROTATE_RIGHT:
      digitalWrite(dir_lf, CW);  
      digitalWrite(dir_rr, CW); 
      digitalWrite(dir_lr, CCW);  
      digitalWrite(dir_rf, CCW);
	  
	  analogWrite (pwm_lf, throttleLeft);
      analogWrite (pwm_lr, throttleLeft);  
      analogWrite (pwm_rr, throttleRight);
      analogWrite (pwm_rf, throttleRight);
      break;
    case DIRECTION_STOP:
      analogWrite(pwm_lf, 0);
      analogWrite(pwm_rr, 0);
      analogWrite(pwm_lr, 0);
      analogWrite(pwm_rf, 0);
      break;
      }
   }

  bUpdateFlags = 0;
}


// simple interrupt service routine
void calcThrottle() {
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH) {
    ulThrottleStart = micros();
   } else {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);

    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
   }
}

void calcSteering(){
  if(digitalRead(STEERING_IN_PIN) == HIGH) {
		ulSteeringStart = micros();
	} else {
		unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
		bUpdateFlagsShared |= STEERING_FLAG;
	}
}





