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
  
  analogWrite (pwm_lf, throttleLeft*lf_mtr_adj);
  analogWrite (pwm_rr, throttleRight*rr_mtr_adj);
  analogWrite (pwm_lr, throttleLeft*lr_mtr_adj);
  analogWrite (pwm_rf, throttleRight*rf_mtr_adj);
}

void mForward() {  
  digitalWrite(dir_lf, CW);  
  digitalWrite(dir_rr, CCW);
  digitalWrite(dir_lr, CCW);    
  digitalWrite(dir_rf, CW);

  analogWrite (pwm_lf, throttleLeft*lf_mtr_adj);
  analogWrite (pwm_rr, throttleRight*rr_mtr_adj);
  analogWrite (pwm_lr, throttleLeft*lr_mtr_adj);
  analogWrite (pwm_rf, throttleRight*rf_mtr_adj);
  
}

void mLeft() {
  digitalWrite(dir_lf, CCW);  
  digitalWrite(dir_rr, CCW); 
  digitalWrite(dir_lr, CW);  
  digitalWrite(dir_rf, CW);
  
  analogWrite (pwm_lf, throttleLeft*lf_mtr_adj);
  analogWrite (pwm_lr, throttleLeft*lr_mtr_adj);
  analogWrite (pwm_rf, throttleRight*rf_mtr_adj);
  analogWrite (pwm_rr, throttleRight*rr_mtr_adj);
}

void mRight()
{
  digitalWrite(dir_lf, CW);  
  digitalWrite(dir_rr, CW); 
  digitalWrite(dir_lr, CCW);  
  digitalWrite(dir_rf, CCW);

  analogWrite (pwm_lf, throttleLeft*lf_mtr_adj);
  analogWrite (pwm_rr, throttleRight*rr_mtr_adj);
  analogWrite (pwm_lr, throttleLeft*lr_mtr_adj);
  analogWrite (pwm_rf, throttleRight*rf_mtr_adj);
  
}

void mStop() {

  analogWrite(pwm_lf, 0);
  analogWrite(pwm_rr, 0);
  analogWrite(pwm_lr, 0);
  analogWrite(pwm_rf, 0);

}

void getTicks_reset(){
  ticksLF = encA.read();
  ticksRF = encB.read();
  ticksRR = encC.read();
  ticksLR = encD.read();
  
  telem.println(ticksLF);  telem.println(ticksRF);
  telem.println(ticksLR);  telem.println(ticksRR);
  encA.write(0); encB.write(0); encC.write(0); encD.write(0);
}

void getTicks_noreset(){
  ticksLF = encA.read();
  ticksRF = encB.read();
  ticksRR = encC.read();
  ticksLR = encD.read();
  
  telem.println(ticksLF);  telem.println(ticksRF);
  telem.println(ticksLR);  telem.println(ticksRR);
  telem.println();
}

void getCurrent() {
      telem << "LF-Current: " << _FLOAT(analogRead(CURRENTLF)*v2Amps,3) << endl;
      telem << "RF-Current: " << _FLOAT(analogRead(CURRENTRF)*v2Amps,3) << endl; 
      telem << "LR-Current: " << _FLOAT(analogRead(CURRENTLR)*v2Amps,3) << endl;
      telem << "RR-Current: " << _FLOAT(analogRead(CURRENTRR)*v2Amps,3) << endl;
	  telem << endl;

}







