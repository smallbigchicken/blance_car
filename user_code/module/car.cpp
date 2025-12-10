#include "Car.h"
#include "main.h" // 包含 CAN 发送函数的头文件




fp32 left=0;
fp32 bias=0;

fp32 target=0.02;

const PidParam PID_UPRIGHT = {
    21000.0f,  
    0.0f,    // 绝对不要给 Ki
    200000.0f,   // 必须给 Kd，否则震荡
    0,
    5000 // 电机最大电流 (16384满)
};


const PidParam PID_SPEED = {
    -2.5f,   // 负号取决于电机安装方向，需调试
    -0.02f,  
    0.0f,    // 速度环一般不用 D
    500,
    15.0f // 限制最大倾角 (例如 15度)，防止加速过猛倒地
};

const PidParam PID_TURN = {
    6.0f,
    0.0f,
    0.5f,
    0,
    4000
};


Car car(can_receive.get_dji_motor_measure_point(0),can_receive.get_dji_motor_measure_point(1),can_receive.get_dm_imu_measure_point(),PID_UPRIGHT,PID_SPEED,PID_TURN,dt7.get_remote_control_point());



Car::Car(const dji_motor_measure_t *left_ptr, const dji_motor_measure_t *right_ptr, const dm_imu_measure_t *imu_ptr, 
    const PidParam &pid_upright, const PidParam &pid_speed, const PidParam &pid_turn,const RC_ctrl_t *gimbal_RC):
    left_leg(left_ptr),right_leg(right_ptr),imu(imu_ptr),pid_upright(PID_POSITION,pid_upright),pid_speed(PID_POSITION,pid_speed),pid_turn(PID_ANGLE,pid_turn),RC(gimbal_RC),
    stop_mode(0)
{

}




// 1. 数据反馈更新
void Car::feedback_update() {

    if(RC->rc.s[1] == 0x02||RC->rc.s[1] == 0x00) stop_mode=true;
    else if (RC->rc.s[1] == 0x03) stop_mode=FALSE;

    imu.update();
    left_leg.update();
    right_leg.update();
    current_pitch = imu.euler[2];
    current_yaw_rate = imu.gyro[2]; // Z轴角速度
    
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

   
    out_balance = pid_upright.Calc(current_pitch, target);

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