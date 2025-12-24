#ifndef MOTOR_H
#define MOTOR_H

#include "can_receive.h"
#include "pid.h"
#define SPEED 0
#define DJI_RPM_TO_RAD 0.10466666 // 2*PI/60*19 因为没有减速箱

class motor
{
public:
    // speed & speed_set rad/s
    float speed;
    float speed_set;

    // speed_PID
    Pid speed_pid;
    float current_give;

    // 构造函数 每个电机都有自己的PID
    motor(const fp32 *speed_parm = NULL);

    virtual void update() = 0;
    void set(float set, uint8_t mode);
    void solve(uint8_t mode);
};

// 继承基本电机类，可以尝试扩展多个牌子的电机
class DJI_Motor : public motor
{
public:
    // 电机反馈信息结构体
    const dji_motor_measure_t *motor_measure;

    DJI_Motor();
    DJI_Motor(const dji_motor_measure_t *motor_ptr,
              const fp32 *speed_parm = NULL);

    void update() override;
};

#endif