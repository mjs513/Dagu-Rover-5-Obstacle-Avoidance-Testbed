#define M_PI 3.14159


//Enable 400Khz I2C bus speed
const boolean fastmode = true;

//compass reads
const int compass_avg_cnt = 20;
const float alpha = 0.5;

float aheading, yar_heading, new_heading, angle_delta;
