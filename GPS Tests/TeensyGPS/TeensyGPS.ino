#include <Streaming.h>
#include <elapsedMillis.h>
#include <TinyGPS++.h>
#include <FilteringScheme.h>

#define telem Serial
#define ss Serial2


//GPS Baud Rate
#define GPSBaud 115200

// The TinyGPS++ object
TinyGPSPlus gps;

KalmanFilter kFilters[4];
int k_index = 2;

/* Taken from TinyGPS
   By declaring TinyGPSCustom objects like this, we announce that we
   are interested in the 15th, 16th, and 17th fields in the $GPGSA 
   sentence, respectively the PDOP (F("positional dilution of precision")),
   HDOP (F("horizontal...")), and VDOP (F("vertical...")).

   (Counting starts with the field immediately following the sentence name, 
   i.e. $GPGSA.  For more information on NMEA sentences, consult your
   GPS module's documentation and/or http://aprs.gids.nl/nmea/.)

   If your GPS module doesn't support the $GPGSA sentence, then you 
   won't get any output from this program.
*/

TinyGPSCustom pdop(gps, "GNGSA", 15); // $GPGSA sentence, 15th element
//TinyGPSCustom vdop(gps, "GNGSA", 17); // $GPGSA sentence, 17th element

elapsedMillis gps_waypoint_timer;
#define defaultWayPointTime 500


void setup(){
	telem.begin(57600);
	telem.println("Rover 5 Obstacle Avoidance");

	ss.begin(GPSBaud);  //GPS Device Baud Rate
	
	delay(100);

  float qVal = 0.125; //Set Q Kalman Filter(process noise) value between 0 and 1
  float rVal = 16.;   //Set K Kalman Filter (sensor noise)
  
  for(int i = 0; i <= k_index; i++) { //Initialize Kalman Filters for 10 neighbors
  //KalmanFilter(float q, float r, float p, float intial_value);
    kFilters[i].KalmanInit(qVal,rVal,10.0,0.5);
  }

  gps_ready();

}

void loop(){
        
        telem << "GPS Fix => ";
        telem.print(gps.location.lat(),6); telem << ",  "; telem.print(gps.location.lng(),6);
        telem <<  ",  " << gps.hdop.value() <<  ",  "  << pdop.value();
        telem << endl;
        smartDelay(1000);
}


//======================================================================


void gps_ready() {

  while(1){
    if(gps.satellites.value() < 5 || gps.satellites.isValid() == 0 || gps.hdop.value() > 150 ||
      gps.hdop.isValid() == 0 || gps.location.isValid() == 0 || atof(pdop.value()) > 2.5) {
        telem << "Acquiring GPS Fix => " << gps.satellites.value() << ",  " << gps.satellites.isValid();
        telem <<  ",  " << gps.hdop.value() <<  ",  " << gps.hdop.isValid() <<  ",  " << pdop.value();
        telem <<   ",  " << gps.location.isValid() << endl;
        smartDelay(1000);
        
        if(telem.available() > 0 ) {
          int val = telem.read();  //read telem input commands  
          if(val == 'p') {
            telem.println("Returning to main"); 
            return;
          }
        }
      } else {
        return;
      }
  }
}


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


