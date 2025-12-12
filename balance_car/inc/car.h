#ifndef CAR_H
#define CAR_H

#include "pid.h"
#include "motor.h"
#include "imu.h"



class Car {
public:
    Car();
    Car(const dji_motor_measure_t* left_ptr,const dji_motor_measure_t* right_ptr,const dm_imu_measure_t* imu_ptr,
        const PidParam &pid_upright,const PidParam &pid_speed,const PidParam &pid_turn);

    // --- 核心任务流函数 ---
    void feedback_update(); // 1. 读取传感器
    void set_control();     // 2. 设定目标 (来自遥控器)
    void solve();           // 3. PID 计算
    void output();          // 4. 发送电流给电机

private:
        


    DJI_Motor left_leg;
    DJI_Motor right_leg;
    Imu imu;

    // PID 对象
    Pid pid_upright;
    Pid pid_speed;
    Pid pid_turn;

    // 状态量
    fp32 current_pitch;
    fp32 current_speed;
    fp32 current_yaw_rate;

    // 目标量
    fp32 target_speed;
    fp32 target_turn;
    
    // 中间计算量
    fp32 target_pitch_angle; // 速度环算出的目标角度

    // 标志位
    bool stop_mode;          // 倒地保护标志
    const fp32 STOP_ANGLE = 40.0f; // 倒地阈值
};

extern Car car; // 声明全局对象
#endif