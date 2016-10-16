#include <Arduino.h>
#include "ubxNMEA.h"
#include <Streaming.h>

//======================================================================
//  Program: NMEAmerged.ino
//
//  Prerequisites:
//     1) NMEA.ino works with your device
//     2) At least one NMEA sentence has been enabled in NMEAGPS_cfg.h
//     3) NMEAGPS_merged is enabled in NMEAGPS_cfg.h
//     3) Explicit or Implicit merging is enabled in NMEAGPS_cfg.h
//
//  Description:  This program guarantees coherency in the fix data.  
//     When a sentence is received with a new time interval, 
//     the 'merged' fix will start with just that new data.  
//     All data from the previous interval is replaced or deleted.  
//     As new sentences are received, data from this new interval 
//     are merged into the 'merged' fix.
//     
//     This program also shows how to 'poll' for a specific message 
//     if it is not sent by default.
//
//  Serial is for debug output to the Serial Monitor window.
//
//======================================================================

#define gps_port Serial2
//#define DEBUG_PORT Serial
#define telem Serial

int val = 0;
volatile byte interrupt = 0;
const int ledPin = 13;

//------------------------------------------------------------
// Check that the config files are set up properly


#if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
    !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
    !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
    !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST ) & \
    !defined( NMEAGPS_PARSE_PUBX_00 ) & !defined( NMEAGPS_PARSE_PUBX_04 )

  #error No NMEA sentences enabled: no fix data available for fusing.
#endif

#if !defined( NMEAGPS_PARSE_PUBX_00 ) & !defined( NMEAGPS_PARSE_PUBX_04 )
 // #error No PUBX messages enabled!  You must enable one or more in ubxNMEA.h!
#endif

#ifndef NMEAGPS_EXPLICIT_MERGING
  #error You must define NMEAGPS_EXPLICIT_MERGING in NMEAGPS_cfg.h
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
  #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

//------------------------------------------------------------


static ubloxNMEA gps         ; // This parses received characters
static gps_fix   merged;

//NeoGPS::Location_t madrid( 407746670L, -738146590L ); 

//----------------------------------------------------------------

static void poll()
{
  gps.send_P( &gps_port, F("PUBX,00") );
  gps.send_P( &gps_port, F("PUBX,04") );
}

//----------------------------------------------------------------

//----------------------------------------------------------------
static void doSomeWork()
{
    //float dist = NeoGPS::Location_t::DistanceKm( merged.location, madrid );
    //float bearing = NeoGPS::Location_t::BearingToDegrees( merged.location, madrid );
    
    //DEBUG_PORT << dist*1000. << ", " << bearing;
    //DEBUG_PORT << endl;

  if(val == 10){
    telem << merged.latitudeL() << "," << merged.longitudeL() << ",";
    telem << merged.hdop << "," << merged.pdop << "," << merged.speed() <<  ","  ;
    telem << merged.heading() << "," <<merged.satellites << ",";
    telem << merged.dateTime << "." << merged.dateTime_cs <<"," << merged.valid.location;
    telem << "," << '\n' << endl;
    
    telem.flush();
    //delay(1000);
     //  Ask for the proprietary messages again
    poll();
     val = 0;
  }
} // doSomeWork

//------------------------------------

static void GPSloop()
{
  if (gps.available( gps_port )) {
    merged = gps.read();

    if(telem.available() > 0){
      val = telem.read();
    }

    //telem << val << endl;
    
    doSomeWork();   
  }

} // GPSloop
  
//--------------------------

void setup()
{

  #ifndef NMEAGPS_PARSE_PUBX_00
    if (LAST_SENTENCE_IN_INTERVAL == (NMEAGPS::nmea_msg_t) ubloxNMEA::PUBX_00) {
      telem.println( F("ERROR! LAST_SENTENCE_IN_INTERVAL PUBX_00 not enabled!\n"
                            "  Either change LAST_SENTENCE or enable PUBX_00")      );
      for(;;);
    }
  #endif
  
  #ifndef NMEAGPS_PARSE_PUBX_04
    if (LAST_SENTENCE_IN_INTERVAL == (NMEAGPS::nmea_msg_t) ubloxNMEA::PUBX_04) {
      telem.println( F("ERROR! LAST_SENTENCE_IN_INTERVAL PUBX_04 not enabled!\n"
                            "  Either change LAST_SENTENCE or enable PUBX_04")      );
      for(;;);
    }
  #endif

  // Start the UART for the GPS device
  gps_port.begin(115200);

  telem.begin(57600);
  telem.flush();

  // Ask for the special PUBX sentences
  poll();

}

//--------------------------

void loop()
{
  GPSloop();

}
