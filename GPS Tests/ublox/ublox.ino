#include <Arduino.h>
#include "ubxGPS.h"

//======================================================================
//  Program: ublox.ino
//
//  Prerequisites:
//     1) You have a ublox GPS device
//     2) PUBX.ino works with your device
//     3) You have installed the ubxGPS.* and ubxmsg.* files.
//     4) At least one UBX message has been enabled in ubxGPS.h.
//     5) Implicit Merging is disabled in NMEAGPS_cfg.h.
//
//  Description:  This program parses UBX binary protocal messages from
//     ublox devices.  It shows how to acquire the information necessary
//     to use the GPS Time-Of-Week in many UBX messages.  As an offset
//     from midnight Sunday morning (GPS time), you also need the current 
//     UTC time (this is *not* GPS time) and the current number of GPS 
//     leap seconds.
//
//  Serial is for debug output to the Serial Monitor window.
//
//======================================================================

#include "Streamers.h"
#include <Streaming.h>

  #define DEBUG_PORT Serial
#define gps_port Serial2
static gps_fix coherent;

//------------------------------------------------------------
// Check that the config files are set up properly

#if !defined(UBLOX_PARSE_STATUS) & !defined(UBLOX_PARSE_TIMEGPS) & \
    !defined(UBLOX_PARSE_TIMEUTC) & !defined(UBLOX_PARSE_POSLLH) & \
    !defined(UBLOX_PARSE_VELNED) & !defined(UBLOX_PARSE_SVINFO)

  #error No UBX binary messages enabled: no fix data available for fusing.

#endif

//-----------------------------------------------------------------
//  Derive a class to add the state machine for starting up:
//    1) The status must change to something other than NONE.
//    2) The GPS leap seconds must be received
//    3) The UTC time must be received
//    4) All configured messages are "requested"
//         (i.e., "enabled" in the ublox device)
//  Then, all configured messages are parsed and explicitly merged.

class MyGPS : public ubloxGPS
{
public:

    enum
      {
        GETTING_STATUS, 
        GETTING_LEAP_SECONDS, 
        GETTING_UTC, 
        RUNNING
      }
        state NEOGPS_BF(8);

    MyGPS( Stream *device ) : ubloxGPS( device )
    {
      state = GETTING_STATUS;
    }

    //--------------------------

    void get_status()
    {
      static bool acquiring = false;

      if (fix().status == gps_fix::STATUS_NONE) {
        static uint32_t dotPrint;
        bool            requestNavStatus = false;

        if (!acquiring) {
          acquiring = true;
          dotPrint = millis();
          DEBUG_PORT.print( F("Acquiring...") );
          requestNavStatus = true;

        } else if (millis() - dotPrint > 1000UL) {
          dotPrint = millis();
          DEBUG_PORT << '.';

          static uint8_t requestPeriod;
          if ((++requestPeriod & 0x07) == 0)
            requestNavStatus = true;
        }

        if (requestNavStatus)
          // Turn on the UBX status message
          enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_STATUS );

      } else {
        if (acquiring)
          DEBUG_PORT << '\n';
        DEBUG_PORT << F("Acquired status: ") << (uint8_t) fix().status << '\n';

        #if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & \
            defined(UBLOX_PARSE_TIMEGPS)

          if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS ))
            DEBUG_PORT.print( F("enable TIMEGPS failed!\n") );

          state = GETTING_LEAP_SECONDS;
        #else
          start_running();
          state = RUNNING;
        #endif
      }
    } // get_status

    //--------------------------

    void get_leap_seconds()
    {
      #if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & \
          defined(UBLOX_PARSE_TIMEGPS)

        if (GPSTime::leap_seconds != 0) {
          DEBUG_PORT << F("Acquired leap seconds: ") << GPSTime::leap_seconds << '\n';

          if (!disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS ))
            DEBUG_PORT.print( F("disable TIMEGPS failed!\n") );

          #if defined(UBLOX_PARSE_TIMEUTC)
            if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
              DEBUG_PORT.print( F("enable TIMEUTC failed!\n") );
            state = GETTING_UTC;
          #else
            start_running();
          #endif
        }
      #endif

    } // get_leap_seconds

    //--------------------------

    void get_utc()
    {
      #if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & \
          defined(UBLOX_PARSE_TIMEUTC)

        lock();
          bool            safe = is_safe();
          NeoGPS::clock_t sow  = GPSTime::start_of_week();
          NeoGPS::time_t  utc  = fix().dateTime;
        unlock();

        if (safe && (sow != 0)) {
          DEBUG_PORT << F("Acquired UTC: ") << utc << '\n';
          DEBUG_PORT << F("Acquired Start-of-Week: ") << sow << '\n';

          start_running();
        }
      #endif

    } // get_utc

    //--------------------------

    void start_running()
    {
      bool enabled_msg_with_time = false;

      #if (defined(GPS_FIX_LOCATION) | \
           defined(GPS_FIX_LOCATION_DMS) | \
           defined(GPS_FIX_ALTITUDE)) & \
          defined(UBLOX_PARSE_POSLLH)
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_POSLLH ))
          DEBUG_PORT.print( F("enable POSLLH failed!\n") );

        enabled_msg_with_time = true;
      #endif

      #if (defined(GPS_FIX_SPEED) | defined(GPS_FIX_HEADING)) & \
          defined(UBLOX_PARSE_VELNED)
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_VELNED ))
          DEBUG_PORT.print( F("enable VELNED failed!\n") );

        enabled_msg_with_time = true;
      #endif

      #if (defined(GPS_FIX_SATELLITES) | defined(NMEAGPS_PARSE_SATELLITES)) & \
          defined(UBLOX_PARSE_SVINFO)
        if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_SVINFO ))
          DEBUG_PORT << PSTR("enable SVINFO failed!\n");
        
        enabled_msg_with_time = true;
      #endif

      #if defined(UBLOX_PARSE_TIMEUTC)

        #if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
          if (enabled_msg_with_time &&
              !disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
            DEBUG_PORT.print( F("disable TIMEUTC failed!\n") );

        #elif defined(GPS_FIX_TIME) | defined(GPS_FIX_DATE)
          // If both aren't defined, we can't convert TOW to UTC,
          // so ask for the separate UTC message.
          if (!enable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC ))
            DEBUG_PORT.print( F("enable TIMEUTC failed!\n") );
        #endif

      #endif

      state = RUNNING;
      trace_header( DEBUG_PORT );

    } // start_running

    //--------------------------

    bool running()
    {
      switch (state) {
        case GETTING_STATUS      : get_status      (); break;
        case GETTING_LEAP_SECONDS: get_leap_seconds(); break;
        case GETTING_UTC         : get_utc         (); break;
      }

      return (state == RUNNING);

    } // running

} NEOGPS_PACKED;

// Construct the GPS object and hook it to the appropriate serial device
static MyGPS gps( &gps_port );

#ifdef NMEAGPS_INTERRUPT_PROCESSING
  static void GPSisr( uint8_t c )
  {
    gps.handle( c );
  }
#endif

//--------------------------

static void configNMEA( uint8_t rate )
{
  for (uint8_t i=NMEAGPS::NMEA_FIRST_MSG; i<=NMEAGPS::NMEA_LAST_MSG; i++) {
    ublox::configNMEA( gps, (NMEAGPS::nmea_msg_t) i, rate );
  }
}

//--------------------------

static void disableUBX()
{
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_VELNED );
  gps.disable_msg( ublox::UBX_NAV, ublox::UBX_NAV_POSLLH );
}

//--------------------------

void setup()
{
  // Start the normal trace output
  DEBUG_PORT.begin(57600);
  while (!DEBUG_PORT)
    ;

  DEBUG_PORT.print( F("ublox binary protocol example started.\n") );
  DEBUG_PORT << F("fix object size = ") << sizeof(gps.fix()) << '\n';
  DEBUG_PORT << F("ubloxGPS object size = ") << sizeof(ubloxGPS) << '\n';
  DEBUG_PORT << F("MyGPS object size = ") << sizeof(gps) << '\n';
  //DEBUG_PORT.println( F("Looking for GPS device on " USING_GPS_PORT) );
  DEBUG_PORT.flush();

  // Start the UART for the GPS device
  gps_port.begin(115200);

  // Turn off the preconfigured NMEA standard messages
  configNMEA( 0 );

  // Turn off things that may be left on by a previous build
  disableUBX();

  #if 1
    // Test a Neo M8 message -- should be rejected by Neo-6 and Neo7
    ublox::cfg_nmea_v1_t test;

    test.always_output_pos  = false; // invalid or failed
    test.output_invalid_pos = false;
    test.output_invalid_time= false;
    test.output_invalid_date= false;
    test.use_GPS_only       = false;
    test.output_heading     = false; // even if frozen
    test.__not_used__       = false;

    test.nmea_version = ublox::cfg_nmea_v1_t::NMEA_V_4_0;
    test.num_sats_per_talker_id = ublox::cfg_nmea_v1_t::SV_PER_TALKERID_UNLIMITED;

    test.compatibility_mode = false;
    test.considering_mode   = true;
    test.max_line_length_82 = false;
    test.__not_used_1__     = 0;

    test.filter_gps    = false;
    test.filter_sbas   = false;
    test.__not_used_2__= 0;
    test.filter_qzss   = false;
    test.filter_glonass= false;
    test.filter_beidou = false;
    test.__not_used_3__= 0;

    test.proprietary_sat_numbering = false;
    test.main_talker_id = ublox::cfg_nmea_v1_t::MAIN_TALKER_ID_GP;
    test.gsv_uses_main_talker_id = true;
    test.beidou_talker_id[0] = 'G';
    test.beidou_talker_id[1] = 'P';

    DEBUG_PORT << F("CFG_NMEA result = ") << gps.send( test );
  #endif

  while (!gps.running())
    if (gps.available( gps_port ))
      gps.read();
}

//--------------------------

void loop()
{
  if (gps.available( gps_port ))
    coherent = gps.read();
    trace_all( DEBUG_PORT, gps, coherent);
    DEBUG_PORT << coherent.hdop << ", " << coherent.pdop << ", " << coherent.speed() <<  ", "  ;
    DEBUG_PORT << coherent.heading() << ", " << coherent.latitudeL() << ", " ;
    DEBUG_PORT << coherent.longitudeL() << ", " << coherent.satellites << ", ";
    DEBUG_PORT << coherent.dateTime << "." << coherent.dateTime_cs <<", " << coherent.valid.location;
    DEBUG_PORT << endl;

  // If the user types something, reset the message configuration
  //   back to a normal set of NMEA messages.  This makes it
  //   convenient to switch to another example program that
  //   expects a typical set of messages.  This also saves
  //   putting those config messages in every other example.

  if (DEBUG_PORT.available()) {
    do { DEBUG_PORT.read(); } while (DEBUG_PORT.available());
    DEBUG_PORT.println( F("Stopping...") );

    configNMEA( 1 );
    disableUBX();
    gps_port.flush();
    gps_port.end();

    DEBUG_PORT.println( F("STOPPED.") );
    for (;;);
  }
}
