void send_odometry(){
    //===  Telemetry section =========
    //if(telem_timer > defaultTelemTime) {

      //DateTime time = rtc.now();
      //telem << time.timestamp(DateTime::TIMESTAMP_TIME);
      //telem << utc << ",";

      telem << etm_millis.elapsed()/1000. << ",";
           
      // IMU
      compass_update();
      telem << -roll << "," << -pitch << "," << yar_heading << ",";

      //Get Current Sense
      getCurrent();

      //Wheel Encoders
      getTicks_noreset();

      /********************************************************************/
      /*                                                                  */
      /*        Begin odometry                                            */
      /*                                                                  */
      /********************************************************************/  

      left_encoder_count = (ticksLF + ticksLR)/2.0;
      right_encoder_count = (ticksRF + ticksRR)/2.0;
      
      // zeros out encoder counts and reads encoders zero value
      //encA.write(0); encB.write(0); encC.write(0); encD.write(0);
      encA.write(0); encB.write(0); encC.write(0); encD.write(0);
      
      float Displacement = (left_encoder_count + right_encoder_count)*ENCODER_SCALE_FACTOR/2.0;
      //float Rotation = (left_encoder_count - right_encoder_count)*ENCODER_SCALE_FACTOR/TRACK;

      pos_x = pos_x + Displacement * cos(yar_heading-init_heading);
      pos_y = pos_y + Displacement * sin(yar_heading-init_heading);

      telem << pos_x << "," << pos_y  << endl;

      //telem_timer = 0;
    //}
}

void odometry(){
  
 ENCODER_SCALE_FACTOR = WHEEL_DIA*PI/CLICKS_PER_REV;
 odo_start_distance = 0;
 odo_start = 0;
 
 while(odo_mode_toggle == 1){
  while (telem.available() > 0) {
    int val = telem.read();  //read telem input commands

    turn_time_mult = telem.parseInt();
    turn_time_mult = turn_time_mult + odo_start_distance;  

    if(turn_time_mult == 0)
                turn_time_mult = 0;          

    if(odo_start == 0){
      odo_start = 1;
    }
    
    odo_start_distance = turn_time_mult;
    
    switch(val)
    {
      case 'f' : 
        odo_timer = 0;
        set_speed(speed);
        gDirection = DIRECTION_FORWARD;
        telem << "Rolling Forward!" << endl;
        telem << turn_time_mult << endl;

        compass_update();
        init_heading = yar_heading;
        etm_millis.start();
        send_odometry();
        mForward();
        
       while(pos_x < turn_time_mult){
        //currentTime = millis();
        if (odo_timer > defaultOdoTime){
          compass_update();
          send_odometry();
          odo_timer = 0;
        }
        mForward();
       }
      //etm_millis.stop(); 
      mStop(); 
      break;
      
    case 't' :
      telem.println("Turning to New Heading");
      set_speed(turnSpeed);
      compass_update();
      new_heading = turn_time_mult + yar_heading;
      pivotToOdo(new_heading);
      mStop();
      break;
             
    case 'b' :
        odo_timer = 0;
        set_speed(speed);
        gDirection = DIRECTION_FORWARD;        
        telem.println("Rolling Forward!");

        mBackward();
        etm_millis.start();
        send_odometry();
        
       while(pos_x < turn_time_mult){
        //currentTime = millis();
        if (odo_timer > defaultOdoTime){
          compass_update();
          send_odometry();
          odo_timer = 0;
        }
        mBackward();
       }
      mStop();
      etm_millis.stop();
      motorRevTime = 0;
      break;
      
    case 's' :      
      telem.println("Stop!");
      mStop();
      break;

    case 'o' :
      telem.println("Toggle Odometry!");
      toggleOdo();
      mStop;
      break;
    }
  }
 }
}


void pivotToOdo(int target){

  int currentAngle = yar_heading;
  int diff = target - currentAngle;
  
  while(abs(diff) > HEADING_TOLERANCE){
    compass_update();
    currentAngle = yar_heading;
    diff = target - currentAngle;

    telem << "Compass Control: " << endl;
    telem << "\t" << currentAngle << ", " << target << ", " << diff << endl;
    if (odo_timer > defaultOdoTime){
      compass_update();
      send_odometry();
      odo_timer = 0;
    }
    
    if(diff > 0) {
      throttleRight = turnSpeed;
      throttleLeft = turnSpeed;
      mRight();//right
    } else {
      throttleRight = turnSpeed;
      throttleLeft = turnSpeed;
      mLeft();//left
    }
    
    send_odometry();
  }
  mStop();
  return;
}

 
