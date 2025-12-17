#include "car.h"




Car::Car(const dji_motor_measure_t* left_motor_ptr,
        const dji_motor_measure_t* right_motor_ptr,
        const dm_imu_measure_t* imu_ptr,
        const fp32* speed_parm):
left_leg(left_motor_ptr,speed_parm),
right_leg(right_motor_ptr,speed_parm),
imu(imu_ptr),L(0.23)
{

}


void Car::calculate_differential_target() {
    // 直接使用类成员变量，无需传参
    // 旋转分量 v_diff = omega * (L / 2)
    float v_diff = this->target_turn * (this->L / 2.0f);

    // 更新成员变量 speeds
    this->speeds.right_velocity = this->target_speed + v_diff;
    this->speeds.left_velocity  = this->target_speed - v_diff;
    

}

// 1. 数据反馈更新
void Car::feedback_update() {

    imu.update();
    left_leg.update();
    right_leg.update();

    
    // 计算平均速度 (RPM 或 m/s，需与PID参数匹配)
    // current_speed = (left_leg.speed_ms + right_leg.speed_ms) / 2.0f;
    // current_yaw_rate = imu.gyro[2];
    // current_pitch = imu.euler[0];
    if(i==300){
        //std::cout<<"当前模式:"<<stop_mode<<std::endl;
        //std::cout<<"当前pitch:"<<current_pitch<<std::endl;
        // std::cout<<"当前速度:"<<current_speed<<std::endl;
        // std::cout<<"当前yaw速度:"<<current_yaw_rate<<std::endl;
        i=0;
    }
    else{
        i++;
    }
}

// 2. 设定控制目标
void Car::set_control(float v, float w) {
    this->target_speed = v;
    this->target_turn = w;  
    calculate_differential_target();
    left_leg.set(-((this->speeds.left_velocity)/0.03),SPEED);
    right_leg.set((this->speeds.right_velocity)/0.03,SPEED);
    
}


void Car::solve() {
  
    left_leg.solve(SPEED);
    right_leg.solve(SPEED);
}

// 4. 硬件输出
void Car::output() {
    
    
    //右轮负电 后退
    //左轮正电 前进
    //if(i==200) std::cout<<"左轮速度："<<left_leg.current_give<<"右轮速度："<<right_leg.current_give<<std::endl;
    can_receive.can_cmd_leg_motor(int(left_leg.current_give),int(right_leg.current_give), CAN_LEGS_ALL_ID);
    //can_receive.can_cmd_leg_motor(0, 0, CAN_LEGS_ALL_ID);
    //can_receive.can_cmd_leg_motor(int(left+bias), int(-left),CAN_LEGS_ALL_ID);
}

void Car::finish()
{
    can_receive.can_cmd_leg_motor(0,0, CAN_LEGS_ALL_ID);
}
