#include "car.h"




Car::Car(const dji_motor_measure_t *left_ptr, const dji_motor_measure_t *right_ptr, const dm_imu_measure_t* imu_ptr,
    const PidParam &pid_upright, const PidParam &pid_speed, const PidParam &pid_turn):
    left_leg(left_ptr),right_leg(right_ptr),imu(imu_ptr),pid_upright(PID_POSITION,pid_upright),pid_speed(PID_POSITION,pid_speed),pid_turn(PID_ANGLE,pid_turn),
    stop_mode(0)
{

}




// 1. 数据反馈更新
void Car::feedback_update() {

    imu.update();
    left_leg.update();
    right_leg.update();

    
    // 计算平均速度 (RPM 或 m/s，需与PID参数匹配)
    current_speed = (left_leg.speed_ms + right_leg.speed_ms) / 2.0f;
    current_yaw_rate = imu.gyro[2];
    current_pitch = imu.euler[0];
    if(i==300){
        std::cout<<"当前模式:"<<stop_mode<<std::endl;
        std::cout<<"当前pitch:"<<current_pitch<<std::endl;
        std::cout<<"当前速度:"<<current_speed<<std::endl;
        std::cout<<"当前yaw速度:"<<current_yaw_rate<<std::endl;
        i=0;
    }
    else{
        i++;
    }
}

// 2. 设定控制目标
void Car::set_control() {
    if (stop_mode) {
        target_speed = 0;
        target_turn = 0;
        return;
    }



    target_speed = 0.1f;
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
    fp32 out_put=0;

    out_put = pid_speed.Calc(current_speed, target_speed);

   
    

    // --- C. 转向环 ---
    // 输入：Yaw角速度，输出：转向力矩
    fp32 out_turn = pid_turn.Calc(current_yaw_rate, target_turn);

    // --- D. 动力分配 ---
     fp32 final_l = out_balance + out_turn;
     fp32 final_r = out_balance - out_turn;

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
    if(i==200) std::cout<<"左轮速度："<<left_leg.current_give<<"右轮速度："<<right_leg.current_give<<std::endl;
    can_receive.can_cmd_leg_motor(left_leg.current_give, -right_leg.current_give, CAN_LEGS_ALL_ID);
    //can_receive.can_cmd_leg_motor(int(left+bias), int(-left),CAN_LEGS_ALL_ID);
}