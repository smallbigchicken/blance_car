#include "Car.h"
#include "main.h" // 包含 CAN 发送函数的头文件

DJI_Motor motor_left;
DJI_Motor motor_right;
Imu imu_data;



Car car;


const PidParam PID_UPRIGHT = {
    .kp = 600.0f,  
    .ki = 0.0f,    // 绝对不要给 Ki
    .kd = 18.0f,   // 必须给 Kd，否则震荡
    .kf = 0.0f,
    .max_iout = 0,
    .max_out = 10000 // 电机最大电流 (16384满)
};


const PidParam PID_SPEED = {
    .kp = -2.5f,   // 负号取决于电机安装方向，需调试
    .ki = -0.02f,  
    .kd = 0.0f,    // 速度环一般不用 D
    .kf = 0.0f,
    .max_iout = 500,
    .max_out = 15.0f // 限制最大倾角 (例如 15度)，防止加速过猛倒地
};

const PidParam PID_TURN = {
    .kp = 6.0f,
    .ki = 0.0f,
    .kd = 0.5f,
    .kf = 0.0f,
    .max_iout = 0,
    .max_out = 4000
};


Car::Car() : stop_mode(true) {}

void Car::init(DJI_Motor* l_ptr, DJI_Motor* r_ptr, Imu* imu_ptr) {
    motor_l = l_ptr;
    motor_r = r_ptr;
    imu = imu_ptr;
}

void Car::pid_init(PidParam upright, PidParam speed, PidParam turn) {
    pid_upright = Pid(PID_POSITION, upright);
    pid_speed   = Pid(PID_POSITION, speed);
    pid_turn    = Pid(PID_POSITION, turn); // 假设控制的是角速度
}

// 1. 数据反馈更新
void Car::feedback_update() {
    
    current_pitch = imu->euler[0]; // 假设 [1] 是 Pitch
    current_yaw_rate = imu->imu_measure->gyro_z; // Z轴角速度
    
    // 计算平均速度 (RPM 或 m/s，需与PID参数匹配)
    current_speed = (motor_l->speed_rpm + motor_r->speed_rpm) / 2.0f;

    // 倒地检测
    if (std::abs(current_pitch) > STOP_ANGLE) {
        stop_mode = true;
    } else {
        stop_mode = false;
    }
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
        
        motor_l->current_give = 0;
        motor_r->current_give = 0;
        return;
    }

    // --- A. 速度环 (外环) ---
    // 输入：速度，输出：目标Pitch角度
    target_pitch_angle = pid_speed.Calc(current_speed, target_speed);

    // --- B. 直立环 (内环) ---
    // 输入：Pitch角度，输出：平衡力矩
    // 注意：目标角度是 速度环输出 + 机械中值(如果有)
    fp32 out_balance = pid_upright.Calc(current_pitch, target_pitch_angle);

    // --- C. 转向环 ---
    // 输入：Yaw角速度，输出：转向力矩
    fp32 out_turn = pid_turn.Calc(current_yaw_rate, target_turn);

    // --- D. 动力分配 ---
    fp32 final_l = out_balance + out_turn;
    fp32 final_r = out_balance - out_turn;

    // 赋值给电机对象 (注意类型转换)
    motor_l->current_give = (int16_t)final_l;
    motor_r->current_give = (int16_t)final_r;
}

// 4. 硬件输出
void Car::output() {
    
    if (stop_mode) {
        motor_l->current_give = 0;
        motor_r->current_give = 0;
    }
    
    
    can_receive.can_cmd_leg_motor(motor_l->current_give, motor_r->current_give,CAN_LEGS_ALL_ID);
}