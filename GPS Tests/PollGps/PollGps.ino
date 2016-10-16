#include <elapsedMillis.h>
//#include <cstring>
#include <Streaming.h>
#include <math.h>

#define mySerial_GPS Serial2
#define defaultTelemTime 1000
elapsedMillis telem_timer;
#define cout Serial

// Function prototype
char *myStrtok(char *parseStr, char *splitChars);
char    tokChars[8]; // Array of characters used to split on
char    *cptr;       // Pointer to string returned by tokenizers
int     count;       // Counter for words in the string

bool first_loop_exec;

#define GPS_INFO_BUFFER_SIZE 256
char GPS_info_char;
char GPS_info_buffer[GPS_INFO_BUFFER_SIZE];
unsigned int received_char;

uint8_t i; // counter
bool message_started;
uint8_t GPS_mess_no = 0;
uint8_t array_idx = 0;
String gpsdata[15];
bool first_gsa = true;
struct utc {
  int mils;
  int seconds;
  int minutes;
  int hours; 
} datetime;

String op_mode;
int fix, sats_in_use, fixtype;
float latlng_age, time_age;
float fix_age, pdop, hdop2, vdop, dop_age, hdop1;
float alt_age, altitude, vert_speed, new_alt;
double longitude, latitude;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Connected");
  mySerial_GPS.begin(57600);
  //mySerial_GPS.println("Connected");

  first_loop_exec=true;
  i=0;
  message_started=false;
      telem_timer = 0;
      strcpy(tokChars, ",");
}

void loop() { // run over and over

  if (first_loop_exec == true){
    delay(2000);
    mySerial_GPS.println(F("$PUBX,40,RMC,0,0,0,0*47")); //RMC OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,VTG,0,0,0,0*5E")); //VTG OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,GGA,0,0,0,0*5A")); //CGA OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,GSA,0,0,0,0*4E")); //GSA OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,GSV,0,0,0,0*59")); //GSV OFF
    delay(100);
    mySerial_GPS.println(F("$PUBX,40,GLL,0,0,0,0*5C")); //GLL OFF
    delay(1000);
    mySerial_GPS.println(F("$PUBX,40,VLW,0,0,0,0*06")); //vlw OFF
    delay(1000);
    first_loop_exec = false;
  }
if(telem_timer > defaultTelemTime) {
      mySerial_GPS.println("$PUBX,00*33"); // data polling to the GPS
      
      mySerial_GPS.println("$EIGNQ,GGA*39");
      //mySerial_GPS.println("$EIGNQ,GSA*2D");
      telem_timer = 0;
}
            read_gps();
}


void read_gps(){
  // MANAGES THE CHARACTERS RECEIVED BY GPS
  while (mySerial_GPS.available()) {
    GPS_info_char = mySerial_GPS.read();
    if (GPS_info_char == '$'){ // start of message
      message_started=true;
      received_char=0;
    }else if (GPS_info_char == '*'){ // end of message
      cout << GPS_mess_no << " ---> " << endl;
      for (i=0; i<received_char; i++){
        Serial.write(GPS_info_buffer[i]); // writes the message to the PC once it has been completely received
      }
      Serial.println();
      

    // Make initial call to strtok() passing in the string to be parsed and
    //        the list of characters used to split tokens apart.
    cptr = strtok(GPS_info_buffer, tokChars);
    count = 1; // Initialize the word counter

    // Create while() loop to print all the tokens in the string.  Note that
    //    the punctuation has been eliminated leaving just the words from the string.
    //    As long as NULL is passed in as the first argument to strtok it will 
    //    continue parsing the last "non-NULL" string passed to it.  It returns
    //    NULL when the entire string has been parsed.
    while(cptr != NULL)
    {
        gpsdata[count] = "";
        for(int len = 0; len < strlen(cptr); len++){
            gpsdata[count] = gpsdata[count]+cptr[len];}
        //cout << "Token " << count << " -->" << gpsdata[count] << "<--\n";
        cptr = strtok(NULL, tokChars); // Get next word
        count++; // Increment counter
    }      
      GPS_mess_no = 0;
      if(strcmp(gpsdata[1].c_str(), "GNGGA") == 0) {
        first_gsa = true;
        GPS_mess_no = 1;
      }
      
      if(strcmp(gpsdata[1].c_str(), "GNGSA") == 0 && first_gsa == true) {
        first_gsa = false;
        GPS_mess_no = 2;
      }
      //cout << gpsdata[1].substring(2,5).c_str() << ">>>>>>> " << GPS_mess_no << endl << endl;
      
      switch(GPS_mess_no){
        case 1:
          read_gga();
          break;
        case 2:
          read_gsa();
          break;
        //do nothing
      }
      message_started=false; // ready for the new message
      
    }else if (message_started==true){ // the message is already started and I got a new character
      if (received_char<=GPS_INFO_BUFFER_SIZE){ // to avoid buffer overflow
        GPS_info_buffer[received_char]=GPS_info_char;
        received_char++;
      }else{ // resets everything (overflow happened)
        message_started=false;
        received_char=0;
      }
    }

  }

  while (Serial.available()) {
    mySerial_GPS.write(Serial.read());
  }
}




