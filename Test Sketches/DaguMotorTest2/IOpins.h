//Motors
const int pwm_lf = 4;  //PWM for CH1 LF
const int pwm_rr = 7;  //PWM for CH4 RR  - Revserved Encoder pins to make postive
const int pwm_lr = 9;  //PWM for CH2 LR
const int pwm_rf = 10;  //PWM for CH3 RF

const int dir_lf = 45;  //DIR for CH1
const int dir_rr = 42;  //DIR for CH4
const int dir_lr = 44;  //DIR for CH2
const int dir_rf = 43;  //DIR for CH3


//Current Sensors
const int CURRENTLF = A12; 
const int CURRENTRR = A13; 
const int CURRENTLR = A14; 
const int CURRENTRF = A15; 
const int CURRENT_LIMIT = (1024 / 5) * 2.6;  // amps

//Encoder Interrupts
//Needs Teensy Encoder library
const int encA1 = 2;
const int encA2 = 46;
const int encB1 = 3;
const int encB2 = 47;
const int encC1 = 18;
const int encC2 = 48;
const int encD1 = 19;
const int encD2 = 49;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encA(encA1, encA2);  // lf
Encoder encB(encB1, encB2);
Encoder encC(encC1, encC2);  // rr
Encoder encD(encD1, encD2);  // lr

