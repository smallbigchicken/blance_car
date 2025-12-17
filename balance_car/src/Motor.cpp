#include "motor.h"
#include <math.h> // for PI

#define PI 3.1415926535f



Motor::Motor(const fp32* speed_parm
    )
    : 
    speed(0), speed_set(0),
    current_give(0),
    speed_pid(speed_parm != NULL ? 
                Pid(0, speed_parm, &speed, &speed_set) : Pid())
{
    
}

DJI_Motor::DJI_Motor(const dji_motor_measure_t* measure_ptr,
                     const fp32* speed_parm
                     ): 
      Motor(speed_parm),
      motor_measure(measure_ptr)
{}


void DJI_Motor::update()
{
    this->speed = motor_measure->speed_rpm * DJI_RPM_TO_RAD;
    //std::cout<<this->speed<<std::endl;
    //std::cout<<motor_measure->speed_rpm<<std::endl;
}



void Motor::set(float set, uint8_t mode)
{
    switch (mode)
    {
    case SPEED:
        this->speed_set = set;
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
    default:
        break;
    }
}