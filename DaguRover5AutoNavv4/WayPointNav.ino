void WaypointNav() {
  //
  // Wait for GPS to get signal
  #ifndef NO_GPS_WAIT
  while (!gps.location.isValid())                      // wait for fix, updating display with each new NMEA sentence received
    {
      // loop until fix established
    } // while (!GPS.fix)
    telem << gps.location.lat() << endl;
  #endif

  telem << "GPS Fix Established" << endl;
  executeWaypointNav();
  
}

void executeWaypointNav(void){
	
	while(waypointNumber < NUMBER_WAYPOINTS) {
	
		nextWaypoint();
		
		// Process GPS 
		processGPS();
		
		// lets pivot to the next waypoint if necessary
		pivotTo(targetHeading);
	
		// navigate 
		compass_update();    // get our current heading
    currentHeading = wp_heading;
		calcDesiredTurn(); 
	
		// distance in front of us, move, and avoid obstacles as necessary
		moveAndAvoid();

		
	}
}

void nextWaypoint(void)
{
  waypointNumber++;
  targetLat = waypointList[waypointNumber].tLat;
  targetLong = waypointList[waypointNumber].tLong;
  
  telem << "Target Lat/Long: " << _FLOAT(targetLat,6) << " , " << _FLOAT(targetLong,6) << endl;
  
  if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
    {
      mStop();    // make sure we stop  
      telem << "***** LAST WAYPOINT *****" << endl;
	  //
	  //
	  // need code to toggle back to command code here
	  //
	  //
    }
    
   processGPS();
   //distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
   
}  // nextWaypoint()




//
// Called after new GPS data is received; updates our position and course/distance to waypoint
void processGPS(void)
{
             
  // update the course and distance to waypoint based on our new position
  double distanceToTarget1 =
    distanceTo(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);

  //course to targetLat
  double targetHeading1 =
      directionTo(
		    gps.location.lat(),
		    gps.location.lng(),
		    targetLat, 
		    targetLong);
  targetHeading = (float) targetHeading1;
  distanceToTarget = distanceToTarget1;
  telem << "process GPS: Target Heading/Distance: " << targetHeading1 << " , " << distanceToTarget1 << endl;
  
}   // processGPS(void)


void calcDesiredTurn(void)
{
    compass_update();
    // calculate where we need to turn to head to destination
    headingError = targetHeading - wp_heading;
    telem << "CalcDesired: Target/Current Heading: " << targetHeading << " , " << wp_heading << endl;
    telem << "Heading Error = " << headingError << endl;
	
    // adjust for compass wrap
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
      turnDirections = 1;  
    else if (headingError < 0)
      turnDirections = 2;
    else if (headingError > 0)
      turnDirections = 3;
    else
      turnDirections = 1;
 
}  // calcDesiredTurn()


void moveAndAvoid(void)
{
	while(distanceToTarget > WAYPOINT_DIST_TOLERANCE) {
		while(abs(headingError) > HEADING_TOLERANCE) {
			if (turnDirections == 1) {
				throttleLeft = throttleRight = FAST_SPEED;
			} else if(turnDirections == 2){
					throttleLeft =  TURN_SPEED;
					throttleRight = TURN_SPEED_DIFF;
				} else if(turnDirections == 3) {
						throttleRight =  TURN_SPEED;
						throttleLeft = TURN_SPEED_DIFF;
					}
      telem << "Direction: " << turnDirections << endl;
			mForward();
			processGPS();
			calcDesiredTurn();
		}
    mForward();
    processGPS();
    calcDesiredTurn();
	}
	
	mStop();  // stop and get next waypoint from list
	
}   // moveAndAvoid()
    

/* static */
double distanceTo(double lat1, double long1, double lat2, double long2)
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

double directionTo(double lat1, double long1, double lat2, double long2)
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
