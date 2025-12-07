#include "Motor.h"

Motor::Motor(const fp32* speed_parm,
    const fp32* encode_parm,
    const fp32* gyro_parm  
    )
    : 
    speed(0), speed_set(0),
    encode_angle(0), encode_angle_set(0),
    gyro_angle(0), gyro_angle_set(0),
    current_give(0),

    
    speed_pid(speed_parm != NULL ? 
                Pid(0, speed_parm, &speed, &speed_set) : Pid()),
    encode_angle_pid(encode_parm != NULL ? 
                Pid(1, encode_parm, &encode_angle, &encode_angle_set) : Pid()),
    gyro_angle_pid(gyro_parm != NULL ? 
                Pid(2, gyro_parm, &gyro_angle, &gyro_angle_set) : Pid())
{
    
}
   
void Motor::update()
{
}

void Motor::set(float set, uint8_t mode)
{
    switch (mode)
    {
    case SPEED:
        this->speed_set = set;
        break;
    case ENCODE_ANGLE:
        this->encode_angle_set = set;
        break;
    case GYRO_ANGLE:
        this->gyro_angle_set = set;
        break;
    default:
        break;
    }
}


void Motor::solve(uint8_t mode)
{
    switch (mode)
    {
    case SPEED:
        current_give = speed_pid.pid_calc();
        break;
    case ENCODE_ANGLE:
        speed_set = encode_angle_pid.pid_calc();
        current_give = speed_pid.pid_calc();
        break;
    case GYRO_ANGLE:
        speed_set = gyro_angle_pid.pid_calc();
        current_give = speed_pid.pid_calc();
        break;
    default:
        break;
    }
}


DJI_Motor::DJI_Motor(const dji_motor_measure_t* measure_ptr,
                     uint16_t offset,
                     uint16_t max,
                     const fp32* speed_parm,
                     const fp32* encode_parm, 
                     const fp32* gyro_parm 
                     ): 
      Motor(speed_parm,encode_parm,gyro_parm),
      motor_measure(measure_ptr), 
      offset_ecd(offset),
      max_ecd(max)
{}


fp32 DJI_Motor::ecd_to_angle(uint16_t ecd)
{
    uint16_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > max_ecd/2)
    {
        relative_ecd -= max_ecd;
    }
    else if (relative_ecd < -max_ecd/2)
    {
        relative_ecd += max_ecd;
    }

    return relative_ecd * 2 * PI / max_ecd;
}

void DJI_Motor::update()
{
    
    this->speed = motor_measure->speed_rpm * DJI_RPM_TO_RAD;
    this->encode_angle = ecd_to_angle(motor_measure->ecd);
}



 
