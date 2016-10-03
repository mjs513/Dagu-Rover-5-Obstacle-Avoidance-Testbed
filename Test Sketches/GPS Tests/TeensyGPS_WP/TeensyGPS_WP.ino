#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Streaming.h>
#include <elapsedMillis.h>
#include <FilteringScheme.h>

#include <Wire.h>
#include <EEPROM.h>

#define telem Serial
#define gps Serial2


//GPS Baud Rate
#define GPSBaud 115200


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

// global for heading from compass
float yar_heading;
#define DEC_ANGLE -13.1603  // -13.1603 degrees for Flushing, NY

elapsedMillis gps_waypoint_timer;
#define defaultWayPointTime 100

/* Set the delay between fresh samples for BNO055*/
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//Globals
float roll, pitch;

//GPS Globals
float currentLat, currentLong;
int hdop, pdop, sv, gps_valid;
float cog, sog;
String utc, gpsdata;
int readonce, val1;

// Compass navigation
float targetHeading,              // where we want to go to reach current waypoint
	  currentHeading,             // where we are actually facing now
	  headingError;               // signed (+/-) difference between targetHeading and currentHeading

// GPS Navigation
float wp_heading,
      distanceToTarget,            // current distance to target (current waypoint)
	    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it

// Waypoints  Constants
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

#define WAYPOINT_DIST_TOLERANCE  3   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 6          // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()

struct waypoints {
    float tLat;
    float tLong;
};

waypoints waypointList[NUMBER_WAYPOINTS] = {
                                            {40.774674, -73.814664},
                                            {40.774664, -73.814612},
                                            {40.774630, -73.814614},
                                            {40.774631, -73.814622},
                                            {40.774614, -73.814669},
                                            {0,0}
                                            };

// ***** End Waypoint Navigation globals
float  targetLat = 40.774674;
float targetLong = -73.814664;

void setup(){
	telem.begin(57600);
	telem.println("Rover 5 Obstacle Avoidance");

	Wire.begin();
	gps.begin(GPSBaud);  //GPS Device Baud Rate
	
	//BNO055_Init();
	delay(100);

  float qVal = 0.125; //Set Q Kalman Filter(process noise) value between 0 and 1
  float rVal = 16.;   //Set K Kalman Filter (sensor noise)
  
  for(int i = 0; i <= k_index; i++) { //Initialize Kalman Filters for 10 neighbors
  //KalmanFilter(float q, float r, float p, float intial_value);
    kFilters[i].KalmanInit(qVal,rVal,10.0,0.5);
  }
  
  readonce = 0;
  gps_ready();

}

void loop(){
  if(gps_waypoint_timer > defaultWayPointTime) {
		//compass_update();
		processGPS();
    gps_waypoint_timer =0;
	}

}


//======================================================================


void compass_update() {
    sensors_event_t event;
    bno.getEvent(&event);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    roll = (float)event.orientation.y;
    pitch = (float)event.orientation.z;
    yar_heading = (float)event.orientation.x;

	// Adjust heading to account for declination
    wp_heading = yar_heading;
    wp_heading += DEC_ANGLE;
    
    //telem << "Compass Yar/dec Heading: " << yar_heading << " , " << heading << endl;
    
    // Correct for when signs are reversed.
    if(wp_heading < 0)
      wp_heading += 360.;
    
    // Check for wrap due to addition of declination.
    if(wp_heading > 360.)
      wp_heading -= 360.;
    
    //telem << roll << "\t" << pitch << "\t" << yar_heading << endl;
    telem << "Changed heading: " << yar_heading << endl;
    
}  
  

//
// Called after new GPS data is received; updates our position and course/distance to waypoint
void processGPS(void)
{
  //float gps_Lat = kFilters[0].measureRSSI(gps.location.lat());
  //float gps_Lng = kFilters[1].measureRSSI(gps.location.lng());
  read_gps();
  // update the course and distance to waypoint based on our new position
  float distanceToTarget1 =
    distanceBetween(
      currentLat,
      currentLong,
      targetLat, 
      targetLong);

  //course to targetLat
   float targetHeading1 =
      courseTo(
		    currentLat,
		    currentLong,
		    targetLat, 
		    targetLong);
        
  targetHeading = targetHeading1;
  distanceToTarget = distanceToTarget1;

  telem << "process GPS: Target Heading/Distance: "   << targetHeading1 << " , " << distanceToTarget1 << endl;
  telem  <<endl;
  
  //if(telem.available() > 0) {
	//  int val = telem.read();  //read telem input commands  
	//  if(val == 'w') {
	//	  toggleWP();
	//	  executeWaypointNav();
	//  }
  //}
}   // processGPS(void)

void read_gps(){
    val1 = 1;
    gps.write(val1);
    gps.flush();
    if(readonce == 0) {
        gpsdata = gps.readString();
        readonce = 1;

        currentLat = atol(getValue(gpsdata, ',', 0).c_str())/10000000.;
        currentLong = atol(getValue(gpsdata, ',', 1).c_str())/10000000.;
        hdop = atoi(getValue(gpsdata, ',', 2).c_str());
        pdop = atoi(getValue(gpsdata, ',', 3).c_str());
        sv = atoi(getValue(gpsdata, ',', 4).c_str());
        sog = atof(getValue(gpsdata, ',', 5).c_str());
        cog = atoi(getValue(gpsdata, ',', 6).c_str());
        utc = getValue(gpsdata, ',', 7).c_str();
        gps_valid = atoi(getValue(gpsdata, ',', 8).c_str());

    telem << _FLOAT(currentLat, 6) << "," << _FLOAT(currentLong, 6) << ",";
    telem << hdop << "," << pdop << "," << sog <<  ","  ;
    telem << cog << "," << sv << ",";
    telem << utc << "," << gps_valid;
    telem << endl << endl;
    }

    //telem_timer = 0;
     readonce = 0;
  //}
  //}
  val1 = 0;
  
}


void BNO055_Init() {
  
    delay(1000);
    telem.println("Orientation Sensor Test"); telem.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        telem.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        telem.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        telem.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        telem.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        telem.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    if (foundCalib){
        telem.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        telem.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            telem.print("X: ");
            telem.print(event.orientation.x, 4);
            telem.print("\tY: ");
            telem.print(event.orientation.y, 4);
            telem.print("\tZ: ");
            telem.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            telem.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    telem.println("\nFully calibrated!");
    telem.println("--------------------------------");
    telem.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    telem.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    telem.println("Data stored to EEPROM.");

    telem.println("\n--------------------------------\n");
    delay(500);
    
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  telem.println("------------------------------------");
  telem.print  ("Sensor:       "); telem.println(sensor.name);
  telem.print  ("Driver Ver:   "); telem.println(sensor.version);
  telem.print  ("Unique ID:    "); telem.println(sensor.sensor_id);
  telem.print  ("Max Value:    "); telem.print(sensor.max_value); telem.println(" xxx");
  telem.print  ("Min Value:    "); telem.print(sensor.min_value); telem.println(" xxx");
  telem.print  ("Resolution:   "); telem.print(sensor.resolution); telem.println(" xxx");
  telem.println("------------------------------------");
  telem.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the telem Monitor */
  telem.println("");
  telem.print("System Status: 0x");
  telem.println(system_status, HEX);
  telem.print("Self Test:     0x");
  telem.println(self_test_results, HEX);
  telem.print("System Error:  0x");
  telem.println(system_error, HEX);
  telem.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  telem.print("\t");
  if (!system)
  {
    telem.print("! ");
  }

  /* Display the individual values */
  telem.print("Sys:");
  telem.print(system, DEC);
  telem.print(" G:");
  telem.print(gyro, DEC);
  telem.print(" A:");
  telem.print(accel, DEC);
  telem.print(" M:");
  telem.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    telem.print("Accelerometer: ");
    telem.print(calibData.accel_offset_x); telem.print(" ");
    telem.print(calibData.accel_offset_y); telem.print(" ");
    telem.print(calibData.accel_offset_z); telem.print(" ");

    telem.print("\nGyro: ");
    telem.print(calibData.gyro_offset_x); telem.print(" ");
    telem.print(calibData.gyro_offset_y); telem.print(" ");
    telem.print(calibData.gyro_offset_z); telem.print(" ");

    telem.print("\nMag: ");
    telem.print(calibData.mag_offset_x); telem.print(" ");
    telem.print(calibData.mag_offset_y); telem.print(" ");
    telem.print(calibData.mag_offset_z); telem.print(" ");

    telem.print("\nAccel Radius: ");
    telem.print(calibData.accel_radius);

    telem.print("\nMag Radius: ");
    telem.print(calibData.mag_radius);
}

void gps_ready() {
  read_gps();
    if(sv < 5 || hdop > 210 ||
      gps_valid == 0 || pdop > 300) {
        telem << "Acquiring GPS Fix => " << sv << ",  " ;
        telem <<  ",  " << hdop <<  ",  " << pdop;
        telem <<   ",  " << gps_valid << endl;
        //smartDelay(1000);
        
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




 String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/* static */
double distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}
