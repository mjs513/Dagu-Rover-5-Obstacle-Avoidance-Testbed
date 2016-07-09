#define M_PI 3.14159


//Enable 400Khz I2C bus speed
const boolean fastmode = true;

//compass reads
const int compass_avg_cnt = 20;
const float alpha = 0.5;

// the interval in mS 
//#define interval 7500    //was 7500
#define interval 100
#define interval1 2000
#define defaultTurnTime 1000
#define defaultFwdTime 7000
#define defaultRevTime 700

float aheading, yar_heading, new_heading, angle_delta;
