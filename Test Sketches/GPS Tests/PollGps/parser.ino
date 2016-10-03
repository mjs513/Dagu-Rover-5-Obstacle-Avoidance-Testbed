// GNGGA 
void read_gga()
{
    cout << "---------------------------" << endl;
    for(int counter = 1; counter <= count; counter++)
    {

        switch(counter)
        {
        case 1: //time
        {
            float time1 = atof(gpsdata[2].c_str());
            float hms = atof(gpsdata[2].substring(0,(gpsdata[2].length()-3)).c_str());
            datetime.mils = (time1 - hms)*100;
            datetime.seconds = fmod(hms, 100);
            hms /= 100;
            datetime.minutes = fmod(hms, 100);
            hms /= 100;
            datetime.hours = hms;

            time_age = millis();
        }
        break;
        case 2: //latitude
        {
            float llat = atof(gpsdata[3].c_str());
            int ilat = llat/100;
            double mins = fmod(llat, 100);
            latitude = ilat + (mins/60);
        }
        break;
        case 3: //north/south
        {
            if(strcmp(gpsdata[4].c_str(),"S") == 0)
                latitude = -latitude;
        }
        break;
        case 4: //longitude
        {
            float llong = atof(gpsdata[5].c_str());
            int ilat = llong/100;
            double mins = fmod(llong, 100);
            longitude = ilat + (mins/60);
        }
        break;
        case 5: //east/west
        {
            if(strcmp(gpsdata[6].c_str(),"W") == 0)
                longitude = -longitude;
            latlng_age = millis();
        }
        break;
        case 6:
        {
            fixtype = (atoi(gpsdata[7].c_str()));
        }
        break;
        case 7:
        {
            sats_in_use = atoi(gpsdata[8].c_str());
        }
        break;
        case 8:
        {
            hdop1 = (float) strtod(gpsdata[9].c_str(), NULL);
        }
        break;
        case 9:
        {
            float new_alt = atof(gpsdata[10].c_str());
            vert_speed = (new_alt - altitude)/((millis()-alt_age)/1000.0);
            altitude = atof(gpsdata[10].c_str());
            alt_age = millis();
        }
        break;
        }
    }
      cout << sats_in_use << ", " << datetime.hours << ":" << datetime.minutes << ":";
      cout << datetime.seconds << "." << datetime.mils << ",";
      cout << _FLOAT(latitude,6) << ", " << _FLOAT(longitude,6) << ", " << hdop2 << ", " << pdop;
      cout << ", " << altitude <<  endl;

}


void read_gsa()
{

//operating mode
            if(strcmp(gpsdata[2].c_str(),"A") == 0)
                //op_mode = MODE_AUTOMATIC;
                op_mode = "A";
            if(strcmp(gpsdata[2].c_str(),"M") == 0)
                //op_mode = MODE_MANUAL;
                op_mode = "M";

            fix = (atoi(gpsdata[3].c_str()));
            fix_age = millis();

            pdop = atof(gpsdata[11].c_str());
 
            hdop2 = atof(gpsdata[12].c_str());

            vdop = atof(gpsdata[13].c_str());
            dop_age = millis();

      //cout << op_mode << ", " << pdop << ", " << hdop2 << endl;

}


