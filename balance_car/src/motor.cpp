#include "motor.h"
#include <math.h> // for PI

#define PI 3.1415926535f

motor::motor(const fp32 *speed_parm)
    : speed(0), speed_set(0),
      current_give(0),
      speed_pid(speed_parm != NULL ? Pid(0, speed_parm, &speed, &speed_set) : Pid())
{
}

DJI_Motor::DJI_Motor(const dji_motor_measure_t *measure_ptr,
                     const fp32 *speed_parm) : motor(speed_parm),
                                               motor_measure(measure_ptr)
{
}

void DJI_Motor::update()
{
    // 做速度控制，仅需更新电机速度
    this->speed = motor_measure->speed_rpm * DJI_RPM_TO_RAD;
}

void motor::set(float set, uint8_t mode)
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

void motor::solve(uint8_t mode)
{
    switch (mode)
    {
    case SPEED: // pid解算
        current_give = speed_pid.pid_calc();
        break;
    default:
        break;
    }
}