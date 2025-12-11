#include "car.h"




Car::Car(const dji_motor_measure_t *left_ptr, const dji_motor_measure_t *right_ptr, 
    const PidParam &pid_upright, const PidParam &pid_speed, const PidParam &pid_turn):
    left_leg(left_ptr),right_leg(right_ptr),pid_upright(PID_POSITION,pid_upright),pid_speed(PID_POSITION,pid_speed),pid_turn(PID_ANGLE,pid_turn),
    stop_mode(1)
{

}




// 1. 数据反馈更新
void Car::feedback_update() {


    left_leg.update();
    right_leg.update();

    
    // 计算平均速度 (RPM 或 m/s，需与PID参数匹配)
    current_speed = (left_leg.speed_rpm + right_leg.speed_rpm) / 2.0f;

    // // 倒地检测
    // if (std::abs(current_pitch) > STOP_ANGLE) {
    //     stop_mode = true;
    // } else {
    //     stop_mode = false;
    // }
}

// 2. 设定控制目标
void Car::set_control() {
    if (stop_mode) {
        target_speed = 0;
        target_turn = 0;
        return;
    }



    target_speed = 0.0f;
    target_turn  = 0.0f;
}


void Car::solve() {
  
    if (stop_mode) {
        pid_upright.Reset();
        pid_speed.Reset();
        pid_turn.Reset();
        
        left_leg.current_give = 0;
        right_leg.current_give = 0;
        return;
    }
    fp32 out_balance=0;
    // --- A. 速度环 (外环) ---
    // 输入：速度，输出：目标Pitch角度
    //target_pitch_angle = pid_speed.Calc(current_speed, target_speed);

   
    out_balance = pid_upright.Calc(current_pitch, 0.02);

    // --- C. 转向环 ---
    // 输入：Yaw角速度，输出：转向力矩
    //fp32 out_turn = pid_turn.Calc(current_yaw_rate, target_turn);

    // --- D. 动力分配 ---
    // fp32 final_l = out_balance + out_turn;
    // fp32 final_r = out_balance - out_turn;

    fp32 final_l = out_balance;
    fp32 final_r = out_balance;
    
    left_leg.current_give = (int16_t)final_l;
    right_leg.current_give = (int16_t)final_r;
}

// 4. 硬件输出
void Car::output() {
    
    if (stop_mode) {
        left_leg.current_give = 0;
        right_leg.current_give = 0;
    }
    
    //右轮负电 后退
    //左轮正电 前进
    can_receive.can_cmd_leg_motor(left_leg.current_give, -right_leg.current_give, CAN_LEGS_ALL_ID);
    //can_receive.can_cmd_leg_motor(int(left+bias), int(-left),CAN_LEGS_ALL_ID);
}