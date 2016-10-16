void WaypointNav() {
  //
  // Wait for GPS to get signal
  #ifndef NO_GPS_WAIT
  while (gps_valid == 1)  // wait for fix, updating display with each new NMEA sentence received
    {
      send_telemetry_wp();
      // loop until fix established
    } // while (!GPS.fix)
  #endif

  telem << "GPS Fix Established" << endl;
  
}

void executeWaypointNav(void){
  
	while(waypointNumber < NUMBER_WAYPOINTS && wp_mode_toggle == 1) {
	
		nextWaypoint();
		
		// Process GPS 
		processGPS();
		
		// lets pivot to the next waypoint if necessary
		pivotTo(targetHeading);
	
		// navigate 
		compass_update();    // get our current heading
		currentHeading = wp_heading;
		calcDesiredTurn(); 
	      telem << "Going to move and avoid" << endl;
		// distance in front of us, move, and avoid obstacles as necessary
		moveAndAvoid();
	}
}


void nextWaypoint(void)
{

  telem << "Reached waypoint number " << waypointNumber << " Going to next point" << endl;
  waypointNumber++;
  targetLat = waypointList[waypointNumber].tLat;
  targetLong = waypointList[waypointNumber].tLong;

  telem << "Target Lat/Long: " << _FLOAT(targetLat,6) << " , " << _FLOAT(targetLong,6) << endl;
  
  if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS && wp_mode_toggle == 1)    // last waypoint reached? 
    {
      mStop();    // make sure we stop  
      telem << "***** LAST WAYPOINT *****" << endl; 
      executeWaypointNav;
	}

   processGPS();
   //distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
   
}  // nextWaypoint()


//
// Called after new GPS data is received; updates our position and course/distance to waypoint
void processGPS(void)
{
  //float gps_Lat = kFilters[0].measureRSSI(gps.location.lat());
  //float gps_Lng = kFilters[1].measureRSSI(gps.location.lng());
  gps_read();

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
  telem  << endl;
  
  //telem << "process GPS: Target Heading/Distance: " << gps_valid << ",  " << targetHeading1 << " , " << distanceToTarget1 << endl;
  
  if(telem.available() > 0) {
	  int val = telem.read();  //read telem input commands  
	  if(val == 'w') {
		  toggleWP();
		  executeWaypointNav();
	  }
  }
}   // processGPS(void)


void calcDesiredTurn(void)
{
    compass_update();
    // calculate where we need to turn to head to destination
    headingError = targetHeading - wp_heading;
	
    //telem << "CalcDesired: Target/Current Heading: " << targetHeading << " , " << wp_heading << endl;
    telem << "Heading Error = " << headingError << endl << endl;
	
    // adjust for compass wrap
    //if (headingError < -180)      
    //  headingError += 360;
    //if (headingError > 180)
    //  headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
      turnDirections = 1;  
    else if(headingError < 0)
      turnDirections = 2;
    else if(headingError > 0)
      turnDirections = 3;
     
    telem << "Direction: " << turnDirections << endl;
    
}  // calcDesiredTurn()


void moveAndAvoid(void)
{
	while(distanceToTarget > WAYPOINT_DIST_TOLERANCE && wp_mode_toggle == 1 ) {
      if (turnDirections == 1) {
          set_speed(FAST_SPEED);
          mForward();
      }
      
		while(abs(headingError) > HEADING_TOLERANCE && wp_mode_toggle == 1) {

			if(turnDirections == 2){  
          set_speed(turnSpeed);
          mLeft();
      }

			if(turnDirections == 3) {
          set_speed(turnSpeed);
          mRight();
      }
      
        delay(defaultWayPointTime);
			  processGPS();
			  calcDesiredTurn();
        send_telemetry_wp();
        gps_waypoint_timer = 0;
        
		}
        delay(defaultWayPointTime);
        processGPS();
        calcDesiredTurn();
        send_telemetry_wp();
        gps_waypoint_timer = 0;
	}
	
	mStop();  // stop and get next waypoint from list
	
}   // moveAndAvoid()
    

