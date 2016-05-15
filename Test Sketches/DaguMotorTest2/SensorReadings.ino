void compass_update() {
    for(int i = 0; i < compass_avg_cnt; i++) {
      compass.read();
      //float aheading = compass.heading();
      yamartino.add(compass.heading());
      delay(20);
      //telem.print("Average Wind Direction: ");
      //telem.println(yamartino.averageHeading());
      //telem.println(aheading);
      //telem.println("");
    }
    yar_heading = yamartino.averageHeading();
    //telem << "Changed heading: " << yar_heading << endl;
   
  }  

void getInclination() {
      compass.read();
      
      float pitch, roll, Xg, Yg, Zg;

      Zg = compass.a.y;
      Yg = compass.a.z;
      Xg = compass.a.x;
  
      //Low Pass Filter
      fXg = Xg * alpha + (fXg * (1.0 - alpha));
      fYg = Yg * alpha + (fYg * (1.0 - alpha));
      fZg = Zg * alpha + (fZg * (1.0 - alpha)); 
      
      //Roll & Pitch Equations
      roll  = 90+(atan2(fYg, -fZg)*180.0)/M_PI;  //reverse signs from An3461
      pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI; 

      //telem << endl << pitch << ",  " << roll << endl;

}
