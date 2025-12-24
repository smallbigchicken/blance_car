#include "car.h"

Car::Car(const dji_motor_measure_t *left_motor_ptr,
         const dji_motor_measure_t *right_motor_ptr,
         const fp32 *speed_parm) : left_leg(left_motor_ptr, speed_parm),
                                   right_leg(right_motor_ptr, speed_parm),
                                   L(0.23)
{
}

void Car::calculate_differential_target()
{
    // 直接使用类成员变量，无需传参
    // 旋转分量 v_diff = omega * (L / 2)
    float v_diff = this->target_turn * (this->L / 2.0f);

    // 更新成员变量 speeds
    this->speeds.right_velocity = this->target_speed + v_diff;
    this->speeds.left_velocity = this->target_speed - v_diff;
}

// 数据反馈更新
void Car::feedback_update()
{

    left_leg.update();
    right_leg.update();
}

// 设定控制目标
void Car::set_control(float v, float w)
{
    // v: 目标线速度 (m/s)
    // w: 目标角速度 (rad/s)，顺时针为正
    this->target_speed = -v;
    this->target_turn = w;
    calculate_differential_target();
    left_leg.set(-((this->speeds.left_velocity) / 0.03), SPEED);
    right_leg.set((this->speeds.right_velocity) / 0.03, SPEED);
}

void Car::solve()
{

    left_leg.solve(SPEED);
    right_leg.solve(SPEED);
}

// 硬件输出
void Car::output()
{

    // 右轮负电 后退
    // 左轮正电 前进
    can_receive.can_cmd_leg_motor(int(left_leg.current_give), int(right_leg.current_give), CAN_LEGS_ALL_ID);
}

// 终止函数，发送0信号
void Car::finish()
{
    can_receive.can_cmd_leg_motor(0, 0, CAN_LEGS_ALL_ID);
}
