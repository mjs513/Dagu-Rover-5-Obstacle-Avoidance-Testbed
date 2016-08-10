void WaypointNav() {
  //
  // Wait for GPS to get signal
  #ifndef NO_GPS_WAIT
  while (!gps.location.isValid())                      // wait for fix, updating display with each new NMEA sentence received
    {
      // loop until fix established
    } // while (!GPS.fix)
  #endif
  
  nextWaypoint();
  
  executeWaypointNav();
  
}

void executeWaypointNav(void){
    // Process GPS 
	processGPS();
	
    // navigate 
    currentHeading = readCompass();    // get our current heading
    calcDesiredTurn(); 
	
    // distance in front of us, move, and avoid obstacles as necessary
    moveAndAvoid(); 

}

void nextWaypoint(void)
{
  waypointNumber++;
  targetLat = waypointList[waypointNumber].getLat();
  targetLong = waypointList[waypointNumber].getLong();
  
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
   distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
   
}  // nextWaypoint()




//
// Called after new GPS data is received; updates our position and course/distance to waypoint
void processGPS(void)
{
             
  // update the course and distance to waypoint based on our new position
  float distanceToTarget =
    TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      target_Lat, 
      target_Long);

  //course to targetLat
    float targetHeading =
      TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      target_Lat, 
      target_Long);
  
}   // processGPS(void)


void calcDesiredTurn(void)
{
    // calculate where we need to turn to head to destination
    headingError = targetHeading - currentHeading;
    
    // adjust for compass wrap
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
      turnDirection = straight;  
    else if (headingError < 0)
      turnDirection = left;
    else if (headingError > 0)
      turnDirection = right;
    else
      turnDirection = straight;
 
}  // calcDesiredTurn()


void moveAndAvoid(void)
{
	while(distanceToTarget <= WAYPOINT_DIST_TOLERANE) {
		while(abs(headingError) < HEADING_TOLERANCE) {
			if (turnDirection == straight) {
				turnLeft = turnRight = FAST_SPEED;
			} else if(turnDiretion == left){
					turnLeft =  TURN_SPEED;
					turnRight = TURN_SPEED_DIFF;
				} else if(turnDirection == right) {
						turnRight =  TURN_SPEED;
						turnLeft = TURN_SPEED_DIFF;
					}
			mForward();
			processGPS();
			calcDesiredTurn();
		}
	}
}   // moveAndAvoid()
    

