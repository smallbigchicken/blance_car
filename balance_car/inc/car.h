#ifndef CAR_H
#define CAR_H

#include "pid.h"
#include "motor.h"

struct WheelSpeeds
{
    float left_velocity;  // m/s
    float right_velocity; // m/s
};

class Car
{
public:
    Car();
    Car(const dji_motor_measure_t *left_motor_ptr,
        const dji_motor_measure_t *right_motor_ptr,
        const fp32 *speed_parm = NULL);

    // --- 核心任务流函数 ---
    void feedback_update();             // 更新电机速度
    void set_control(float v, float w); //  设定目标 电机速度
    void solve();                       //  PID 计算
    void output();                      //  发送电流给电机
    void finish();                      //  停止电机输出

    void calculate_differential_target();
    int i = 0;
    DJI_Motor left_leg;
    DJI_Motor right_leg;

    // 当前值
    float current_speed;    // 前进速度
    float current_yaw_rate; // 旋转速度

    // 两轮目标速度
    WheelSpeeds speeds;

    // 目标量
    float target_speed;
    float target_turn;

    // L 轮距
    float L;
};

extern Car car; // 声明全局对象
#endif