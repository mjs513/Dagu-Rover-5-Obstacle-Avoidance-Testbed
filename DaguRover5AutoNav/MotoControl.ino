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

void mBackward()
{  
  digitalWrite(dir_lf, CCW);  
  digitalWrite(dir_rr, CW);
  digitalWrite(dir_lr, CW);    
  digitalWrite(dir_rf, CCW);
  
  analogWrite (pwm_lf, speed);
  analogWrite (pwm_rr, speed);
  analogWrite (pwm_lr, speed);
  analogWrite (pwm_rf, speed);
  
}

void mForward() {  
  digitalWrite(dir_lf, CW);  
  digitalWrite(dir_rr, CCW);
  digitalWrite(dir_lr, CCW);    
  digitalWrite(dir_rf, CW);
  
  analogWrite (pwm_lf, speed);
  analogWrite (pwm_rr, speed);
  analogWrite (pwm_lr, speed);
  analogWrite (pwm_rf, speed);  
  
}

void mLeft() {
  digitalWrite(dir_lf, CCW);  
  digitalWrite(dir_rr, CCW); 
  digitalWrite(dir_lr, CW);  
  digitalWrite(dir_rf, CW);
  
  analogWrite (pwm_lf, turnSpeed);
  analogWrite (pwm_rr, turnSpeed);
  analogWrite (pwm_lr, turnSpeed);
  analogWrite (pwm_rf, turnSpeed);
  
}

void mRight()
{
  digitalWrite(dir_lf, CW);  
  digitalWrite(dir_rr, CW); 
  digitalWrite(dir_lr, CCW);  
  digitalWrite(dir_rf, CCW);
  
  analogWrite (pwm_lf, turnSpeed);
  analogWrite (pwm_rr, turnSpeed);
  analogWrite (pwm_lr, turnSpeed);
  analogWrite (pwm_rf, turnSpeed);
  
}

void mStop() {

  analogWrite(pwm_lf, 0);
  analogWrite(pwm_rr, 0);
  analogWrite(pwm_lr, 0);
  analogWrite(pwm_rf, 0);

}

void getTicks_reset(){
  int ticksLF = encA.read();
  int ticksRF = encB.read();
  int ticksRR = encC.read();
  int ticksLR = encD.read();
  
  telem.println(ticksLF);  telem.println(ticksRF);
  telem.println(ticksLR);  telem.println(ticksRR);
  encA.write(0); encB.write(0); encC.write(0); encD.write(0);
}

void getTicks_noreset(){
  int ticksLF = encA.read();
  int ticksRF = encB.read();
  int ticksRR = encC.read();
  int ticksLR = encD.read();
  
  telem.println(ticksLF);  telem.println(ticksRF);
  telem.println(ticksLR);  telem.println(ticksRR);

}

void getCurrent() {
      telem.print("LF-Current: ");
      telem.println(analogRead(CURRENTLF)*v2Amps);
      
      telem.print("RR-Current: ");
      telem.println(analogRead(CURRENTRR)*v2Amps);
      
      telem.print("LR-Current: ");
      telem.println(analogRead(CURRENTLR)*v2Amps);
      
      telem.print("RF-Current: ");
      telem.println(analogRead(CURRENTRR)*v2Amps);

}
