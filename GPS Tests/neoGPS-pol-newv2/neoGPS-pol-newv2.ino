#include "ubxNMEA.h"
using namespace NeoGPS;

#include <Streaming.h>
#include <math.h>

#define cout Serial
#define gps_port Serial2
#define defaultTelemTime 1500
uint32_t telem_timer;
int count1;
String utc1;

ubloxNMEA gps;
gps_fix   fix;

//------------------------------------------------------------

static gps_fix    first;            // good GPS data
static clock_t    firstSecs;        // cached dateTime in seconds since EPOCH
static Location_t avgLoc;           // gradually-calculated average location
static uint16_t   count;            // number of samples
static int32_t    sumDLat, sumDLon; // accumulated deltas
static bool       doneAccumulating; // accumulation completed

//------------------------------------------------------------

// Use a Finite-state machine (aka FSM) to implement the
//   periodic request of a "$PUBX,00# message.
enum state_t { WAITING_TO_REQ, REQUESTING_PUBX }; // possible states
state_t state;                                    // current state variable

//---------------------------------------------------------

void setup() {

  Serial.begin(57600);

  cout << F("Connected") << endl;

  gps_port.begin(57600);

    gps.send_P( &gps_port, F("PUBX,40,RMC,0,0,0,0")); //RMC OFF
    gps.send_P( &gps_port, F("PUBX,40,VTG,0,0,0,0")); //VTG OFF
    gps.send_P( &gps_port, F("PUBX,40,GGA,0,0,0,0")); //CGA OFF
    gps.send_P( &gps_port, F("PUBX,40,GSA,0,0,0,0")); //GSA OFF
    gps.send_P( &gps_port, F("PUBX,40,GSV,0,0,0,0")); //GSV OFF
    gps.send_P( &gps_port, F("PUBX,40,GLL,0,0,0,0")); //GLL OFF
    gps.send_P( &gps_port, F("PUBX,40,VLW,0,0,0,0")); //vlw OFF
}

//---------------------------------------------------------

void loop() {
  // Are there any commands from the Serial Monitor?

  if (cout.available() > 0){
    char cmd = cout.read();
    if (cmd == 'r') {
      cout << F("request data") << endl;
     for(int jj = 0; jj < 360; jj++){
      gps.send_P( &gps_port, F("PUBX,00") ); // data polling to the GPS
      state = REQUESTING_PUBX;

      //  Is there any data from the GPS?
      while(state == REQUESTING_PUBX){
        while (gps.available( gps_port )) {
          fix  = gps.read();
          if (state == REQUESTING_PUBX) {
            state = WAITING_TO_REQ; // got it!
            //displayGPS();
            GPS_Averaage();
          }
        }
      }
    }
    
    }
    
  }
} // loop


//---------------------------------------------------------

void GPS_Averaage(){
  static bool warned = false; // that we're waiting for a valid location

  //if (fix.valid.location && fix.valid.date && fix.valid.time) {

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
        cout.println();
      }

      cout.print( count );

      if (!doneAccumulating) {

        // Enough time?
        if (((clock_t)fix.dateTime - firstSecs) > 2 * SECONDS_PER_HOUR)
          doneAccumulating = true;
      }

      int32_t dLat, dLon;

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
      }

      cout.print( ',' );
      cout.print( avgLoc.lat() );
      cout.print( ',' );
      cout.print( avgLoc.lon() );
      cout.print( ',' );
      dLat = avgLoc.lat() - fix.location.lat();
      cout.print( dLat );
      cout.print( ',' );
      dLon = avgLoc.lon() - fix.location.lon();
      cout.print( dLon );

      // Calculate the distance from the current fix to the average location
      float avgDistError = avgLoc.DistanceKm( fix.location );
      cout.print( ',' );
      cout.print( avgDistError * 100000.0 ); // cm

      // Calculate the bearing from the current fix to the average location.
      //   NOTE: other libraries will have trouble with this calculation,
      //   because these coordinates are *VERY* close together.  Native
      //   floating-point calculations will not have enough significant
      //   digits.
      float avgBearingErr = fix.location.BearingTo( avgLoc );
      float bearing       = avgBearingErr * Location_t::DEG_PER_RAD;
      cout.print( ',' );
      cout.print( bearing, 6 );

      cout.println();
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


void displayGPS()
{
  cout << F("Fix: ");
  if (fix.valid.status)
    cout << (uint8_t) fix.status;
  cout << endl;

  cout << F("Sats: ");
  if (fix.valid.satellites)
    cout << fix.satellites;
  cout << endl;

  cout << F("UTC: ");
  if (fix.valid.date || fix.valid.time) {
    cout << fix.dateTime << '.';
    if (fix.dateTime_cs < 10)
      cout << '0';
    cout << fix.dateTime_cs << endl;
    utc1 = (fix.dateTime.hours) + ":";
    //utc1 = utc1.concat(":");
    //utc1 = utc1.concat(itoa(fix.dateTime.minutes, buffer, 10));
    cout << utc1 << endl << endl;
  }
  cout << endl;

  cout << F("Loc: ");
  if (fix.valid.location)
    cout << fix.latitudeL() << ',' << fix.longitudeL();
  else
    cout << endl;
  cout << endl;

  cout << F("COG: ");
  if (fix.valid.heading)
    cout << fix.heading_cd();
  cout << endl;

  cout << F("SOG: ");
  if (fix.valid.speed)
    cout << fix.speed_mkn();
  cout << endl;

  cout << F("Alt: ");
  if (fix.valid.altitude)
    cout << fix.altitude_cm();
  cout << endl;

  cout << F("DOP (h,v,p): ");
  if (fix.valid.hdop)
    cout << fix.hdop;
  cout << ',';

  if (fix.valid.vdop)
    cout << fix.vdop;
  cout << ',';

  if (fix.valid.pdop)
    cout << fix.pdop;
  cout << endl;

  cout << endl;

} // displayGPS
