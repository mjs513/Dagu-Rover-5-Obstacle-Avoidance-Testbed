void gps_ready() {
      gps_port.write("$PUBX,00*33\r\n"); // data polling to the GPS
      read_gps();
      count = 1;
    if(sv < 7 || hdop > 2000 ||
       pdop > 2100 || gps_valid == 'NF' || sv == 0) {
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
      } else {
        return;
      }
}


//----------------------------------------------------
void read_gps(){

  // MANAGES THE CHARACTERS RECEIVED BY GPS
 while(count <= 20) {
  while (gps_port.available()) {
    GPS_info_char = gps_port.read();
    if (GPS_info_char == '$'){ // start of message
      message_started=true;
      received_char=0;
    } else if (GPS_info_char == '*'){ // end of message
      //telem <<  " ---> " << endl;
      //for (i=0; i<received_char; i++){
      //  Serial.write(GPS_info_buffer[i]); // writes the message to the PC once it has been completely received
      //}
      //Serial.println();
      

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
        //telem << "Token " << count << " -->" << gpsdata[count] << "<--\n";
        cptr = strtok(NULL, tokChars); // Get next word
        count++; // Increment counter
    }      
      
      read_pubx();

      message_started=false; // ready for the new message
      
    } else if (message_started==true){ // the message is already started and I got a new character
      if (received_char<=GPS_INFO_BUFFER_SIZE){ // to avoid buffer overflow
        //telem << GPS_info_char << endl;
        GPS_info_buffer[received_char]=GPS_info_char;
        received_char++;
      }else{ // resets everything (overflow happened)
        //telem << "gps buffer overflow" << endl;
        message_started=false;
        received_char=0;
      }
    }

  }


 // while (Serial.available()) {
 //   gps_port.write(Serial.read());
 // }
 }
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

//----------------------------------------------
// pubx
void read_pubx()
{
    for(int counter = 1; counter <= count; counter++)
    {

        switch(counter)
        {
        case 1: //time
        {
            char buffer[33];
            char str[80];
            float time1 = atof(gpsdata[3].c_str());
            float hms = atof(gpsdata[3].substring(0,(gpsdata[3].length()-3)).c_str());
            int mils = (time1 - hms)*100;
            int seconds = fmod(hms, 100);
            hms /= 100;
            int minutes = fmod(hms, 100);
            hms /= 100;
            int hours = hms;
            utc = "";
            utc.concat(hours); utc.concat(":");
            utc.concat(minutes); utc.concat(":");
            utc.concat(seconds); utc.concat(".");
            utc.concat(mils);
            //time_age = millis();
        }
        break;
        case 2: //latitude
        {
            float llat = atof(gpsdata[4].c_str());
            int ilat = llat/100;
            double mins = fmod(llat, 100);
            currentLat = ilat + (mins/60);
        }
        break;
        case 3: //north/south
        {
            if(strcmp(gpsdata[5].c_str(),"S") == 0)
                currentLat = -currentLat;
        }
        break;
        case 4: //longitude
        {
            float llong = atof(gpsdata[6].c_str());
            int ilat = llong/100;
            double mins = fmod(llong, 100);
            currentLong = ilat + (mins/60);
        }
        break;
        case 5: //east/west
        {
            if(strcmp(gpsdata[7].c_str(),"W") == 0)
                currentLong = -currentLong;
            //latlng_age = millis();
        }
        break;
    
        case 6:
        {
              float altitude = atof(gpsdata[8].c_str());
        //    alt_age = millis();
       }
        break;
        case 7:
        {
            gps_valid = gpsdata[9].c_str();
        }
        break;
        case 8:
        {
          sog = atof(gpsdata[12].c_str());
        }
        break;
        case 9:
        {
          cog = atof(gpsdata[13].c_str());
        }
        break;

        case 10:
        {
          float vVel = atof(gpsdata[14].c_str());
        }
        case 11:
        {
            //hdop1 = (float) strtod(gpsdata[9].c_str(), NULL);
            hdop = atof(gpsdata[15].c_str());
        }
        break;
        case 12:
        {
            sv = atoi(gpsdata[18].c_str());
        }
        break;
    }
    }
      //telem << gps_valid << "," << sv << ", " << utc << ",";
      //telem << _FLOAT(currentLat,6) << ", " << _FLOAT(currentLong,6) << ", " << hdop << ", ";
      //telem << _FLOAT(sog,3) << ", " << _FLOAT(cog,3) <<  endl;

}

//-----------------------------------------------

//-------------------------------------------------
// Function: myStrtok()
// Purpose; Split a string passed in into tokens
//        using a given string of characters as the
//        split characters.
// Args: parseStr - String to be parsed or NULL if
//            parsing is to continue with the current
//            string.
//       splitChars - Array of characters to use as
//            the split characters.
// Returns: Pointer to next token in the string
//--------------------------------------------------
char *myStrtok(char *parseStr, char *splitChars)
{
    // Note: The first var is declared as "static" this
    //  means that after the initial call to the function it
    //    will retain its last value with each subsequent call
    //    to this function. It is initialized on the very first
    //    call to this function to NULL.
    static char *pStr = NULL; // Pointer to string to be tokenized.

    char *tok;  // Pointer to start of next token
    char *temp; // Misc. use temporary pointer.
    int  found; // Boolean flag to indicate a split character was found

    //----------------------------------------------------------------
    // Case 1: See if a new string to be parsed was passed in.  This
    //            will be true if parseStr is not NULL
    //----------------------------------------------------------------
    if(parseStr != NULL)
        pStr = parseStr; // If yes, then hold the pointer to it

    //----------------------------------------------------------------
    // Case 2: Check to see if the last call to this function returned 
    //            the last token in the string. If pStr is now pointing 
    //            to the NULL terminator this will be true
    //----------------------------------------------------------------
    if(*pStr == '\0') return NULL; // Tell user all tokens have been returned

    //----------------------------------------------------------------
    // Case 3: Find the next token.  
    //        Step 1: Starting from the end of the last token returned
    //            or the beginning of the new string to parse skip any
    //            leading characters that are the same as characters found
    //            in the splitChars array. We also look for the NULL
    //            terminator indicating we have reached the end of parseStr.
    //----------------------------------------------------------------
    found = 0; // Initialize to FALSE
    tok = pStr; // Initialize tok pointer to start current point in parseStr
    // Skip any leading splitChars
    while((!found) && (*tok != '\0'))
    {
        temp = splitChars;    // Point to start of splitChars array
        while(*temp != '\0') // Scan entire splitChars array each time
        {
            if(*tok != *temp) 
            {
                temp++; // Advance to next character in splitChars
            }
            else    // Found a split char
            {
                tok++; // Advance to next character in parseStr
                break; // and end this scan of the splitChars array
            }
        }
        // Check to see if we made it through the entire splitChars
        // array without finding a match, i.e. we have the first char
        // in the next token
        if(*temp == '\0') found = 1; // Mark as TRUE to end search
        // Note: If tok was advanced to point to the NULL terminator at the
        //   end of parseStr this will also terminate the loop
    }

    // Check to see if we reached the end of parseStr without finding another
    //        token.  If so set pStr so we can recognize this at the next call
    if(*tok == '\0')
    {
        pStr = tok; // Point pStr to the NULL terminator at the end of parseStr
        return tok; // Return NULL to indicate the end of the string was reached.
    }

    // When we reach this point tok points to the first non-splitChars character

    //----------------------------------------------------------------
    //        Step 2: Find the end of this token.  This will be the next
    //            occurance of one of the characters in splitChars or the
    //            NULL terminator marking the end of parseStr
    //----------------------------------------------------------------
    found = 0; // Initialize to FALSE
    pStr = tok; // Initialize pStr to tok
    // Search for first occurance of a splitChars character marking the end
    //    of this token.  Also look to see if we reach the end of parseStr
    while((!found) && (*pStr != '\0'))
    {
        temp = splitChars;    // Point to start of splitChars array
        // Scan entire splitChars array to see if the char pStr points to
        // is one of the split chars.
        while((*temp != '\0') && (*pStr != *temp)) temp++;

        // if this char was OK advance to the next and try again
        if(*temp == '\0') 
            pStr++;
        else
            found = 1;    // Found the end of the token so end the while() loop
        // Note: If pStr was advanced to point to the NULL terminator at the
        //   end of parseStr this will also terminate the loop
    }

    // At this point we have tok pointing to the first character of the
    //    next token in parseStr and pStr pointing to the first character
    //    after the end of the next token.

    //----------------------------------------------------------------
    //        Step 3: Set up for the return and next call.
    //----------------------------------------------------------------
    // When we reach this point if pStr is pointing to the NULL terminator
    //    at the end of parseStr we leave pStr pointing to this NULL terminator
    //    so we will know this on the next call to this function. 

    if(*pStr != '\0')
    {
        // However, if pStr is not pointing to a NULL terminator then it
        //    must be pointing to a split character so we replace the character 
        //    pStr is pointing to with a NULL terminator so the caller can get 
        //    the token by itself and advance pStr to the first character after 
        //    that so it is ready to parse the next token on the next call to 
        //    this function.
        *pStr = '\0'; // Put a NULL terminator at the end of this token
        pStr++; // Advance pStr to the next character in parseStr
    }

    return 0; // Return the pointer to the next token
}

