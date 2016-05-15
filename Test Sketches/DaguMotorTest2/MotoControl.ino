
//Dagu4Motor motor1(pwm_a, dir_a, CURRENTA); 
//Dagu4Motor motor2(pwm_b, dir_b, CURRENTB); 
//Dagu4Motor motor3(pwm_c, dir_c, CURRENTC); 
//Dagu4Motor motor4(pwm_d, dir_d, CURRENTD); 

void mReverse(int speed)
{  
  digitalWrite(dir_lf, CCW);  
  digitalWrite(dir_rr, CW);
  digitalWrite(dir_lr, CW);    
  digitalWrite(dir_rf, CCW);
  
  analogWrite (pwm_lf, speed);
  analogWrite (pwm_rr, speed);
  analogWrite (pwm_lr, speed);
  analogWrite (pwm_rf, speed);
  
  boolMove = true;

}


void mForward(int speed)
{
  
  digitalWrite(dir_lf, CW);  
  digitalWrite(dir_rr, CCW);
  digitalWrite(dir_lr, CCW);    
  digitalWrite(dir_rf, CW);
  
  analogWrite (pwm_lf, speed);
  analogWrite (pwm_rr, speed);
  analogWrite (pwm_lr, speed);
  analogWrite (pwm_rf, speed);  
  
  boolMove = true;    

}

void mLeft(int speed)
{
  digitalWrite(dir_lf, CCW);  
  digitalWrite(dir_rr, CCW); 
  digitalWrite(dir_lr, CW);  
  digitalWrite(dir_rf, CW);
  
  analogWrite (pwm_lf, speed);
  analogWrite (pwm_rr, speed);
  analogWrite (pwm_lr, speed);
  analogWrite (pwm_rf, speed);
  
  boolMove = true;
}

void mRight(int speed)
{
  digitalWrite(dir_lf, CW);  
  digitalWrite(dir_rr, CW); 
  digitalWrite(dir_lr, CCW);  
  digitalWrite(dir_rf, CCW);
  
  analogWrite (pwm_lf, speed);
  analogWrite (pwm_rr, speed);
  analogWrite (pwm_lr, speed);
  analogWrite (pwm_rf, speed);
  
  boolMove = true;

}

void mStop()
{

  analogWrite(pwm_lf, 0);
  analogWrite(pwm_rr, 0);
  analogWrite(pwm_lr, 0);
  analogWrite(pwm_rf, 0);  
  boolMove = false;

  int ticksA = encA.read();
  int ticksB = encB.read();
  int ticksC = encC.read();
  int ticksD = encD.read();
  
  telem.println(ticksA);  telem.println(ticksB);
  telem.println(ticksC);  telem.println(ticksD);
  encA.write(0); encB.write(0); encC.write(0); encD.write(0);
  
}


