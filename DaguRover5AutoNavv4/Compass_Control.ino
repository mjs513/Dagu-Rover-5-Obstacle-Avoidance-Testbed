/*
  Copyright (c) 2013 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/
void pointTo(int angle){
  int target = angle;
  target = target % 360;
  
  if(target < 0){
    target += 360;
  }
  
  int direction = angle;
  
  while(1){
    compass_update();
    int currentAngle = yar_heading;
    int diff = target - currentAngle;
    direction = 180 - (diff + 360) % 360;
	
    if(direction > 0){
      mRight();//right
      delay(10);
    } else {
      mLeft();//left
      delay(10);
    }
    //if(diff<-180) 
    //  diff += 360;
    //else if(diff> 180) 
    //  diff -= 360;
    //direction=-diff;
    
    if(abs(diff) < 5){
      mStop();
      return;
    }
  }
}

void turn(int angle){
  compass_update();
  int originalAngle = yar_heading;
  int target = originalAngle + angle;
  pointTo(target);
  
  /*uint8_t speed=80;
  target=target%360;
  if(target<0){
    target+=360;
  }
  int direction=angle;
  while(1){
    if(direction>0){
      motorsWrite(speed,speed);//right
      delay(10);
    }else{
      motorsWrite(-speed,-speed);//left
      delay(10);
    }
    int currentAngle=compassRead();
    int diff=target-currentAngle;
    if(diff<-180) 
      diff += 360;
    else if(diff> 180) 
      diff -= 360;
    direction=-diff;
    
    if(abs(diff)<5){
      motorsWrite(0,0);
      return;
    }
  }*/
}












