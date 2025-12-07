#include "motor.h"
#include <math.h> // for PI

#ifndef PI
#define PI 3.1415926535f
#endif


Motor::Motor() {
    speed_rpm = 0;
    speed_rads = 0;
    angle_single_round = 0;
    total_angle = 0;
    current_give = 0;
}


DJI_Motor::DJI_Motor() : Motor() {
    motor_measure = NULL;
    offset_ecd = 0;
    max_ecd = 8191;
    last_ecd = 0;
    round_count = 0;
}

DJI_Motor::DJI_Motor(const dji_motor_measure_t* measure_ptr,
                     uint16_t offset,
                     uint16_t max) 
                     : Motor() // 调用父类构造
{
    motor_measure = measure_ptr;
    offset_ecd = offset;
    max_ecd = max;
    
    // 初始化记录值，防止上电第一帧数据导致圈数误判
    if (measure_ptr != NULL) {
        last_ecd = measure_ptr->ecd;
    } else {
        last_ecd = 0;
    }
    round_count = 0;
}

void DJI_Motor::update()
{
    if (motor_measure == NULL) return;

    
    uint16_t now_ecd = motor_measure->ecd;
    int16_t now_speed_rpm = motor_measure->speed_rpm; // DJI直接给的是RPM

   
    this->speed_rpm = (float)now_speed_rpm;
    this->speed_rads = this->speed_rpm * (2.0f * PI / 60.0f); // RPM -> rad/s

    
    int diff = now_ecd - last_ecd;
    
    if (diff > max_ecd / 2) {
        
        round_count--; 
    } else if (diff < -(max_ecd / 2)) {
        
        round_count++;
    }
    
    last_ecd = now_ecd; 

   
    float total_rounds = (float)round_count + (float)now_ecd / (float)(max_ecd + 1);
    this->total_angle = total_rounds * 2.0f * PI;
    
   
    this->angle_single_round = ((float)now_ecd / (float)(max_ecd + 1)) * 2.0f * PI;
}