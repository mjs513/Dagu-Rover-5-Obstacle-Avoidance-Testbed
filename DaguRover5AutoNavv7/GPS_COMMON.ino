void gps_ready() {
        read_gps();
        displayGPS2();
    while(sv < 7 || hdop > 2.000 ||
       gps_valid == 0) {
        read_gps();
        displayGPS();
        //displayGPS2();
        telem << "Acquiring GPS Fix => " << sv << ",  " ;
        telem <<  hdop <<  ",  " << pdop;
        telem <<   ",  " << gps_valid << endl;
        
        if(telem.available() > 0 ) {
          int val = telem.read();  //read telem input commands  
          if(val == 'p') {
            telem.println("Returning to main"); 
            return;
          }
        }
      }
}


//----------------------------------------------------
void read_gps(){
  gps.send_P( &gps_port, F("PUBX,00") ); // data polling to the GPS
  state = REQUESTING_PUBX;

  //  Is there any data from the GPS?
  while(state == REQUESTING_PUBX){
    while (gps.available( gps_port )) {
      fix  = gps.read();
      if (state == REQUESTING_PUBX) {
        state = WAITING_TO_REQ; // got it!      }
    }
   }
  }

  if (fix.valid.status)
    gps_valid = (uint8_t) fix.status;

  if (fix.valid.satellites)
    sv = fix.satellites;

  //if (fix.valid.date || fix.valid.time) {
    //utc = String(fix.dateTime.year) + "-" + String(fix.dateTime.month) + "-" + String(fix.dateTime.date);
  //  utc = String(fix.dateTime.hours) + ":" + String(fix.dateTime.minutes) + ":";
  //  utc = utc + String(fix.dateTime.seconds) + "." + String(fix.dateTime_cs);
    //if (fix.dateTime_cs < 10)
     // cout << '0';
 //}

  if (fix.valid.location) {
    currentLat = fix.latitudeL();
    currentLong = fix.longitudeL();
  } else {
    currentLat = 0;
    currentLong = 0;
  }

  if (fix.valid.heading)
    cog = fix.heading_cd()/100.;


  if (fix.valid.speed)
    sog = fix.speed_kph();

  if (fix.valid.altitude)
    altitude = (float) fix.altitude_cm()/1000.;

  if (fix.valid.hdop)
    hdop = (float) fix.hdop/1000.;

  //if (fix.valid.vdop)
  //  cout << fix.vdop;
  //telem << ',';

  if (fix.valid.pdop)
    pdop = (float) fix.pdop/1000.;

  //displayGPS();
}


void GPS_Average(){
  static bool warned = false; // that we're waiting for a valid location
  int32_t dLat, dLon;
  
  //if (fix.valid.location && fix.valid.date && fix.valid.time) {
  for(int jj = 0; jj < 180; jj++){

  gps.send_P( &gps_port, F("PUBX,00") ); // data polling to the GPS
  state = REQUESTING_PUBX;

  //  Is there any data from the GPS?
  while(state == REQUESTING_PUBX){
    while (gps.available( gps_port )) {
      fix  = gps.read();
      if (state == REQUESTING_PUBX) {
        state = WAITING_TO_REQ; // got it!      }
    if (count == 0) {
      // Just save the first good fix
      first = fix;
      firstSecs = (clock_t) first.dateTime;
      count = 1;

    } else {

      // After the first fix, accumulate locations until we have
      //   a good average.  Then display the offset from the average.

      if (warned) {
        // We were waiting for the fix to be re-acquired.
        warned = false;
        //telem.println();
      }

      //telem.print( count );

      if (!doneAccumulating) {

        // Enough time?
        if (((clock_t)fix.dateTime - firstSecs) > 2 * SECONDS_PER_HOUR)
          doneAccumulating = true;
      }

      if (!doneAccumulating) {

        // Use deltas from the first location
        dLat = fix.location.lat() - first.location.lat();
        sumDLat += dLat;
        int32_t avgDLat = sumDLat / count;

        dLon = fix.location.lon() - first.location.lon();
        sumDLon += dLon;
        int32_t avgDLon = sumDLon / count;

        //  Then calculated the average location as the first location
        //     plus the averaged deltas.
        avgLoc.lat( first.location.lat() + avgDLat );
        avgLoc.lon( first.location.lon() + avgDLon );

        count++;
    
      //telem.print(jj); telem.print(", ");
      //telem.print( avgLoc.lat() );
      //telem.print( ',' );
      //telem.print( avgLoc.lon() );
      //telem.print( ',' );
      dLat = avgLoc.lat() - fix.location.lat();
      //telem.print( dLat );
      //telem.print( ',' );
      
      dLon = avgLoc.lon() - fix.location.lon();
      //telem.print( dLon );

      // Calculate the distance from the current fix to the average location
      //float avgDistError = avgLoc.DistanceKm( fix.location );
      //telem.print( ',' );
      //telem.print( avgDistError * 100000.0 ); // cm

      // Calculate the bearing from the current fix to the average location.
      //   NOTE: other libraries will have trouble with this calculation,
      //   because these coordinates are *VERY* close together.  Native
      //   floating-point calculations will not have enough significant
      //   digits.
      //float avgBearingErr = fix.location.BearingTo( avgLoc );
      //float bearing       = avgBearingErr * Location_t::DEG_PER_RAD;
      //telem.print( ',' );
      //telem.print( bearing, 6 );

      //telem.println();
      }
    }

  //} else {
  //  if (!warned) {
  //    warned = true;
  //    cout.print( F("Waiting for fix...") );
  //  } else {
  //    cout.print( '.' );
  // }
  //}
    }
   }
  }
  

  }
  
  currentLat = avgLoc.lat();
  currentLong = avgLoc.lon();

}


void displayGPS()
{
  telem << F("Fix: ");
  if (fix.valid.status)
    telem << (uint8_t) fix.status;
  telem << endl;

  telem << F("Sats: ");
  if (fix.valid.satellites)
    telem << fix.satellites;
  telem << endl;

  telem << F("UTC: ");
  if (fix.valid.date || fix.valid.time) {
    telem << fix.dateTime << '.';
    if (fix.dateTime_cs < 10)
      telem << '0';
    telem << fix.dateTime_cs;
  }
  telem << endl;

  telem << F("Loc: ");
  if (fix.valid.location)
    telem << fix.latitudeL() << ',' << fix.longitudeL();
  else
    telem << endl;
  telem << endl;

  telem << F("COG: ");
  if (fix.valid.heading)
    telem << fix.heading_cd();
  telem << endl;

  telem << F("SOG: ");
  if (fix.valid.speed)
    telem << fix.speed_mkn();
  telem << endl;

  telem << F("Alt: ");
  if (fix.valid.altitude)
    telem << fix.altitude_cm();
  telem << endl;

  telem << F("DOP (h,v,p): ");
  if (fix.valid.hdop)
    telem << fix.hdop;
  telem << ',';

  if (fix.valid.vdop)
    telem << fix.vdop;
  telem << ',';

  if (fix.valid.pdop)
    telem << fix.pdop;
  telem << endl;

  telem << endl;

} // displayGPS

void displayGPS2()
{
  telem << F("Fix: ");
    telem << gps_valid;
  telem << endl;

  telem << F("Sats: ");
    telem << sv;
  telem << endl;

  telem << F("UTC: ");
      telem << utc;
  telem << endl;

  telem << F("Loc: ");
    telem << currentLat << ',' << currentLong;
  telem << endl;

  telem << F("COG: ");
    telem << cog;
  telem << endl;

  telem << F("SOG: ");
    telem << sog;
  telem << endl;

  telem << F("Alt: ");
    telem << altitude;
  telem << endl;

  telem << F("DOP (h,v,p): ");
    telem << hdop;
  telem << ',';
  //  telem << vdop;
  //telem << ',';

    telem << pdop;
  telem << endl;

  telem << endl;

} // displayGPS
