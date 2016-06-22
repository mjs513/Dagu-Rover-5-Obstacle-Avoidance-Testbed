//============================================================================
//    Sketch to test various technicques in robotic car design such as
//    obstacle detection and avoidance, compass as turn guide,
//    motor control, etc.
//    Copyright (C) 2015  Michael J Smorto
//    https://github.com/mjs513/Sainsmart_4WD_Obstacle_Avoid_TestBed.git
//    FreeIMU@gmail.com
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with this program; if not, write to the Free Software Foundation, Inc.,
//    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

void decide_direction() {
   
	// No forward obstacles
	if(cm[3] > fowardheadThreshold && cm[0] > sideSensorThreshold && 
        cm[1] > lcThreshold && cm[2] > sideSensorThreshold &&
		    frtIRdistance > lcThreshold) {
		nextMove = "Straight";
		telem << "(DC) Next Move Straight" << endl;
	}
	
	// If everything is blocked in the forward direction lets backupSensorThreshold
	// and run the Bubble rebound/VFH routing
	else if(cm[0] < sideSensorThreshold && cm[2] < sideSensorThreshold && 
			(cm[3] < fowardheadThreshold || cm[1] < lcThreshold 
			|| frtIRdistance < lcIRthreshold)) {
		
		//If I have to move backward 1st test to see to see if there is a obstacle
		//if no obstacle move a lot, if obstacle move a little and then run the bubble
		//rebound algorithm
		if(rearIRdistance > backupSensorThreshold) {
			mBackward();
			delay(backup_high);
			mStop();
			lastMove == "Backup";
		} else {
			mBackward();
			delay(backup_low);
			mStop();
			lastMove == "Backup";
		}    
		nextMove = "RunBubble";
		telem << "(DC) Everything blocked Next Move Backup" << endl;
	}
	
	// Do any of the front facing range sensors detect an obstacle closer than their
	// threshold?  If so, then prepare to turn left or right. cm[3] < fowardheadThreshold ||
	//else if( cm[1] < lcThreshold || frtIRdistance < lcIRthreshold)
		
	// If head sensor is blocked (less than threshold) run bubble and be done with iter_swap
	//else if(cm[3] < fowardheadThreshold) {
	//	nextMove = "RunBubble";
	//}
	
	else if(frtIRdistance < lcIRthreshold || cm[1] < fowardheadThreshold
		|| cm[3] < fowardheadThreshold)	
	{
		//If I have to move backward 1st test to see to see if there is a obstacle
		//if no obstacle move a lot, if obstacle move a little and then run the bubble
		//rebound algorithm
		if(rearIRdistance > backupSensorThreshold) {
			mBackward();
			delay(backup_high);
			mStop();
			lastMove == "Backup";
		} else {
			mBackward();
			delay(backup_low);		//was 250
			mStop();
			lastMove == "Backup";
		}   
		//nextMove = "RunBubble";

		if(cm[0] >= cm[2]) {				//If left  is greater than right distance move left
				nextMove = "Left";
				turn_time = left_57;  	//was 37
				telem << "(DC) Next Move is left (Center Blocked)" << endl;
				//mLeft();
				//delay(turn_time);     //was 1500, 700, 225 - calc at 275 change to 325
				//mStop();
				//nextMove = "Straight";
		} else if(cm[2] >= cm[0]){		//If right is greater than left distance move right
				nextMove = "Right";
				turn_time = right_57; //was 37
				telem << "(DC) Next Move is right (Center Blocked)" << endl;
				//mRight();
				//delay(turn_time);     //was 1500, 700, 225 - calc at 275 change to 325
				//mStop();
				//nextMove = "Straight";
		} else {						//if all else fails run bubble band algorithm
			nextMove = "RunBubble";
			telem << "(DC) Next Move Determined by bubble [fwd sensors blocked - backup first]" << endl;

		}
	}

	// What about the angled looking  detectors?
	// If right and center distances is less than threshold move left alot
	else if(cm[2] < sideSensorThreshold && cm[1] < fowardheadThreshold){
		nextMove = "Left";
		turn_time = left_57;
		telem << "(DC) Next Move alot Left" << endl;
	}
	
	// If left and center distances is less than threshold move right alot
	else if(cm[0] < sideSensorThreshold && cm[1] < fowardheadThreshold){
		nextMove = "Right";
		turn_time = right_57;
		telem << "(DC) Next Move alot Right" << endl;
	}
	
	// If right distances is less than threshold and center not blocked 
	// move left a little
	else if (cm[2] < sideSensorThreshold && cm[1] > fowardheadThreshold)
	{
		nextMove = "Left";
		turn_time = left_37;
		telem << "(DC) Next Move Left" << endl;
	}
	//
	// If left distance is less than threshold and center not blocked 
	// move right a little
	//
	else if(cm[0] < sideSensorThreshold && cm[1] > fowardheadThreshold)
	{
		nextMove = "Right";
		turn_time = right_37;
		telem << "(DC) Next Move Right" << endl;
	}

	// Reset the closest obstacle distance to a large value.
	// minDistance = 10000;
  
	// If the closest object is too close, then get ready to back up.
	//if (minDistance < backupSensorThreshold) nextMove = "Backup";
  
	// If we can't go forward, stop and take the appropriate evasive action.
	if (nextMove != "Straight")
	{
		// lets stop and figure out what's next
		mStop();
		
		if(lastMove == "Right" && nextMove == "Left") {
			nextMove == "Right";
			turn_time = right_57;
			turnCounter = turnCounter + 1;
			telem << "(OSC) Next Move Right" << endl;
		 } else if(lastMove == "Left" && nextMove == "Right") {
			nextMove == "Left";
			turn_time = left_57;
			turnCounter = turnCounter + 1;
			telem << "(OSC) Next Move Left" << endl;
		  }
		if(turnCounter != 0){
			nextMove = lastMove;
		}
		
		if (nextMove == "Right") {
            //nextTurn = -1;  //turn CCW
			mRight();
			//delay(turn_time);     //was 1500, 700, 225 - calc at 275 change to 325
			while(turn_timer < turn_time){
				getCurrent();
			}
			turn_timer = 0;
			mStop();			
		 } else if (nextMove == "Left") { 
			  //nextTurn = -1;  //turn CCW
			  mLeft();
			  //delay(turn_time);     //was 1500, 700, 225 - calc at 275 change to 325
			  while(turn_timer < turn_time){
				getCurrent();
			  }
			  turn_timer = 0;
			  mStop();	        
			  nextTurn = 1;	  //turn CW
		   } else if (nextMove == "RunBubble") {
				turnCounter = 0;
				Select_Direction();
			 }
		   
		lastMove = nextMove;		
    
    } else {
		// If no obstacles are detected close by, keep going straight ahead.
		lastMove = "Straight";

		//reset counters after clearing obstacle
        turnCounter = 0;
		
		mForward();
       
		//keep moving forward until next obstacle is found
		while(cm[3] > fowardheadThreshold && cm[0] > sideSensorThreshold && 
                cm[1] > lcThreshold && cm[2] > sideSensorThreshold &&
                frtIRdistance > lcIRthreshold) {
			if(telem.available() > 0) {
				int val = telem.read();  //read telem input commands  
				if(val == 't') {
					telem.println("toggle Roam Mode"); 
					toggleRoam();
					return;
				}
			}
			telem << "(DC) End of Tests Updating Sensors" << endl << endl;          
			read_sensors();   
			oneSensorCycle();          
		}    
		mStop();
    }
}   



