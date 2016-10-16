// pubx
void read_pubx()
{
    cout << "---------------------------" << endl;
    for(int counter = 1; counter <= count; counter++)
    {

        switch(counter)
        {
        case 1: //time
        {
            float time1 = atof(gpsdata[3].c_str());
            float hms = atof(gpsdata[3].substring(0,(gpsdata[3].length()-3)).c_str());
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
            float llat = atof(gpsdata[4].c_str());
            int ilat = llat/100;
            double mins = fmod(llat, 100);
            latitude = ilat + (mins/60);
        }
        break;
        case 3: //north/south
        {
            if(strcmp(gpsdata[5].c_str(),"S") == 0)
                latitude = -latitude;
        }
        break;
        case 4: //longitude
        {
            float llong = atof(gpsdata[6].c_str());
            int ilat = llong/100;
            double mins = fmod(llong, 100);
            longitude = ilat + (mins/60);
        }
        break;
        case 5: //east/west
        {
            if(strcmp(gpsdata[7].c_str(),"W") == 0)
                longitude = -longitude;
            latlng_age = millis();
        }
        break;
        case 6:
        {
            altitude = atof(gpsdata[8].c_str());
            alt_age = millis();
        }
        break;
        case 7:
        {
            fixtype = gpsdata[9].c_str();
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
          vVel = atof(gpsdata[14].c_str());
        }
        case 11:
        {
            //hdop1 = (float) strtod(gpsdata[9].c_str(), NULL);
            hdop1 = atof(gpsdata[15].c_str());
        }
        break;
        case 12:
        {
            sats_in_use = atoi(gpsdata[18].c_str());
        }
        break;
    }
    }
      cout << fixtype<< "," << sats_in_use << ", " << datetime.hours << ":" << datetime.minutes << ":";
      cout << datetime.seconds << "." << datetime.mils << ",";
      cout << _FLOAT(latitude,6) << ", " << _FLOAT(longitude,6) << ", " << hdop1 << ", ";
      cout << _FLOAT(sog,3) << ", " << _FLOAT(cog,3) << "," << altitude <<  endl;
}
