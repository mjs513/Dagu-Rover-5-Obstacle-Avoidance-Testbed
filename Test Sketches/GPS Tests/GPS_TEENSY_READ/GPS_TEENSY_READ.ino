#include <elapsedMillis.h>
#include <Streaming.h>

#define DEBUG_PORT Serial

elapsedMillis telem_timer;
#define defaultTelemTime 400

float latitude, longitude;
int hdop, pdop, sv, gps_valid;
float cog, sog;
String utc;
String gpsdata;
int readonce;
int val;

void setup() {
  Serial.begin(57600);
  Serial2.begin(115200);
  readonce = 0;
  telem_timer = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  //if(telem_timer > defaultTelemTime) {
    val = 1;
    Serial2.write(val);
    //Serial2.flush();
    
  //if(Serial2.available() > 0){

    if(readonce == 0) {
        gpsdata = Serial2.readString();
        readonce = 1;

        latitude = atol(getValue(gpsdata, ',', 0).c_str())/10000000.;
        longitude = atol(getValue(gpsdata, ',', 1).c_str())/10000000.;
        hdop = atoi(getValue(gpsdata, ',', 2).c_str());
        pdop = atoi(getValue(gpsdata, ',', 3).c_str());
        sv = atoi(getValue(gpsdata, ',', 4).c_str());
        sog = atof(getValue(gpsdata, ',', 5).c_str());
        cog = atoi(getValue(gpsdata, ',', 6).c_str());
        utc = getValue(gpsdata, ',', 7).c_str();
        gps_valid = atoi(getValue(gpsdata, ',', 8).c_str());

    DEBUG_PORT << _FLOAT(latitude, 6) << "," << _FLOAT(longitude, 6) << ",";
    DEBUG_PORT << hdop << "," << pdop << "," << sog <<  ","  ;
    DEBUG_PORT << cog << "," << sv << ",";
    DEBUG_PORT << utc << "," << gps_valid;
    DEBUG_PORT << endl << endl;
    }
    
    telem_timer = 0;
     readonce = 0;
  //}
  //}
  val = 0;

double distanceToTarget1 =
    distanceBetween(
        latitude,
        longitude,
        40.774674, 
        -73.814664);

  //course to targetLat
  double targetHeading1 =
      courseTo(
        latitude,
        longitude,
        40.774674, 
        -73.814664);
 DEBUG_PORT << distanceToTarget1 << ", " << targetHeading1 << endl << endl;
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
