#include <Servo.h> 
#include <NewPing.h>
#include <Streaming.h>

Servo panServo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
int angle = 0;
const int pulse0Degrees = 1000;   // pulse width for 0 degrees
const int pulse360Degrees = 2100;  // pulse width for 360 degrees
                                   //was2100
const int pcenter = 1000; // using 1000 for center, or 0 degrees, you may want to use something different

 
unsigned int pos1 = 0;    // variable to store the servo position 

#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  panServo.attach(9);  // attaches the servo on pin 9 to the servo object 
  panServo.writeMicroseconds(pulse0Degrees);
}

void loop() {
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  for(int i = 0; i < 15; ++i) //goes from 180 degrees to 0 degrees 
  { //145 40
    //Serial << map(pos, 20, 145, 0, 180) << "/ ";
    
    angle = i * 15;
    int pulse = map(angle,0,360,pulse0Degrees, pulse360Degrees);
    panServo.writeMicroseconds(pulse);
    delay(550);               // waits 15ms for the servo to reach the position 
   
  unsigned int uS = sonar.ping_median(); // Send ping, get ping time in microseconds (uS).
  //unsigned int uS = sonar.ping_median(); 
  Serial << ( uS / US_ROUNDTRIP_CM) << ", " << angle << ", " << pulse;
  uS = -999;
  Serial << endl; 
  } 
  //panServo.writeMicroseconds(pulse0Degrees);
  delay(750);
  Serial << endl;
  
  
}


int getIndexOfMaximumValue(int* array, int size){
  int maxIndex = 0;
  int max = array[maxIndex];
  for (int i=1; i<size; i++){
    if (max<array[i]){
      max = array[i];
      maxIndex = i;
    }
  }
  return maxIndex;
}

