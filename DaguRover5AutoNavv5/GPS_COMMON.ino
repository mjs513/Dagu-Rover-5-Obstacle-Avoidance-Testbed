void gps_ready() {
  gps_read();
    if(sv < 7 || hdop > 2000 ||
      gps_valid == 0 || pdop > 2100) {
        telem << "Acquiring GPS Fix => " << sv << ",  " ;
        telem <<  ",  " << hdop <<  ",  " << pdop;
        telem <<   ",  " << gps_valid << endl;
        
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

void gps_read(){
    val1 = 1;
    gps.write(val1);
    gps.flush();
    if(readonce == 0) {
        //if(gps.available() > 0){
            gpsdata = "";
            char character;
            while(gps.available()) {
               character = gps.read();
                gpsdata.concat(character);
            }
        }

        readonce = 1;

        currentLat = atol(getValue(gpsdata, ',', 0).c_str())/10000000.;
        currentLong = atol(getValue(gpsdata, ',', 1).c_str())/10000000.;
        hdop = atoi(getValue(gpsdata, ',', 2).c_str());
        pdop = atoi(getValue(gpsdata, ',', 3).c_str());
        sv = atoi(getValue(gpsdata, ',', 6).c_str());
        sog = atof(getValue(gpsdata, ',', 5).c_str());
        cog = atoi(getValue(gpsdata, ',', 4).c_str());
        utc = getValue(gpsdata, ',', 7).c_str();
        gps_valid = atoi(getValue(gpsdata, ',', 8).c_str());

    //telem << _FLOAT(currentLat, 6) << "," << _FLOAT(currentLong, 6) << ",";
    //telem << hdop/1000 << "," << pdop/1000 << "," << sog <<  ","  ;
    //telem << cog << "," << sv << ",";
    //telem << utc << "," << gps_valid;
    //telem << endl << endl;
    //}

    //telem_timer = 0;
     readonce = 0;
  //}
  //}
  val1 = 0;
  
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
