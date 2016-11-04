#include "ubxNMEA.h"

#include <Streaming.h>
#include <math.h>

#define cout Serial
#define gps_port Serial2
#define defaultTelemTime 1500
uint32_t telem_timer;

ubloxNMEA gps;
gps_fix   fix;

// Use a Finite-state machine (aka FSM) to implement the
//   periodic request of a "$PUBX,00# message.
enum state_t { WAITING_TO_REQ, REQUESTING_PUBX }; // possible states
state_t state;                                    // current state variable

//---------------------------------------------------------

void setup() {

  Serial.begin(57600);
  while (!Serial)
    ;

  cout << F("Connected") << endl;

  gps_port.begin(57600);

  configMessages( false );
}

//---------------------------------------------------------

void loop() {

   bool timeToPoll = (millis() - telem_timer > defaultTelemTime);
  //bool timeToPoll = false;

  // Are there any commands from the Serial Monitor?

  if (cout.available() > 0){
    char cmd = cout.read();

    if (cmd == 'x') {
      // Toggle all the other messages on or off
      static bool onOff = false;
      onOff = !onOff; // toggle
      configMessages( onOff );

    } else if (cmd == 'r') {
      if (state == WAITING_TO_REQ)
        timeToPoll = true;
    }
  }

  // Is it time to request a PUBX,00 message?

  if (timeToPoll) {
    cout << F("request data") << endl;
    gps.send_P( &gps_port, F("PUBX,00") ); // data polling to the GPS
    state         = REQUESTING_PUBX;
    telem_timer   = millis();
  }

  //  Is there any data from the GPS?

  while (gps.available( gps_port )) {
    fix  = gps.read();

    if (state == REQUESTING_PUBX) {
      state = WAITING_TO_REQ; // got it!
      displayGPS();
    }
  }

  //  Did we timeout getting a PUBX response?

  if ((state == REQUESTING_PUBX) && (millis() - telem_timer > 3000L)) {
    cout << F("No GPS response!") << endl;
    state = WAITING_TO_REQ; // go back to waiting
  }

} // loop

//---------------------------------------------------------

void configMessages( bool on )
{
  if (on) {
    gps.send_P( &gps_port, F("PUBX,40,RMC,0,1,0,0")); //RMC ON
    gps.send_P( &gps_port, F("PUBX,40,VTG,0,1,0,0")); //VTG ON
    gps.send_P( &gps_port, F("PUBX,40,GGA,0,1,0,0")); //CGA ON
    gps.send_P( &gps_port, F("PUBX,40,GSA,0,1,0,0")); //GSA ON
    gps.send_P( &gps_port, F("PUBX,40,GSV,0,1,0,0")); //GSV ON
    gps.send_P( &gps_port, F("PUBX,40,GLL,0,1,0,0")); //GLL ON
    gps.send_P( &gps_port, F("PUBX,40,VLW,0,1,0,0")); //vlw ON
  } else {
    gps.send_P( &gps_port, F("PUBX,40,RMC,0,0,0,0")); //RMC OFF
    gps.send_P( &gps_port, F("PUBX,40,VTG,0,0,0,0")); //VTG OFF
    gps.send_P( &gps_port, F("PUBX,40,GGA,0,0,0,0")); //CGA OFF
    gps.send_P( &gps_port, F("PUBX,40,GSA,0,0,0,0")); //GSA OFF
    gps.send_P( &gps_port, F("PUBX,40,GSV,0,0,0,0")); //GSV OFF
    gps.send_P( &gps_port, F("PUBX,40,GLL,0,0,0,0")); //GLL OFF
    gps.send_P( &gps_port, F("PUBX,40,VLW,0,0,0,0")); //vlw OFF
  }
}

//---------------------------------------------------------

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
    cout << fix.dateTime_cs;
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
