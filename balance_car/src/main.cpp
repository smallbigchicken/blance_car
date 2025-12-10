#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include "car.h"
using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

// ================= 配置参数 =================
#define COMMUNICATE_TASK_INIT_TIME  30
#define COMMUNICATE_CONTROL_TIME_MS 2
#define BALANCE_CAR_TASK_INIT_TIME_MS   200
#define BALANCE_CAR_CONTROL_TIME_MS      2

const PidParam PID_UPRIGHT = {
    21000.0f,  
    0.0f,    
    200000.0f,   
    0,
    5000 
};

const PidParam PID_SPEED = {
    -2.5f,   
    -0.02f,  
    0.0f,    
    500,
    15.0f 
};

const PidParam PID_TURN = {
    6.0f,
    0.0f,
    0.5f,
    0,
    4000
};




Car car(can_receive.get_dji_motor_measure_point(0),can_receive.get_dji_motor_measure_point(1),PID_UPRIGHT,PID_SPEED,PID_TURN);


// ================= 全局互斥锁 (Method 3 核心) =================
mutex xGlobalDataMutex; 


void receive_thread_func() {
    while (true) {
        can_receive.receive_once(); 
    }
}


// // ================= 任务 1: 通信任务 =================
// void communicate_Task() // C++ 线程函数通常不需要 void* 参数
// {
   
//     sleep_for(milliseconds(COMMUNICATE_TASK_INIT_TIME));

//     communicate.init();

//     while (true)
//     {
//         communicate.run();
        
//         sleep_for(milliseconds(COMMUNICATE_CONTROL_TIME_MS));
//     }
// }

// ================= 任务 2: 平衡控制任务 =================
void balance_Task()
{
   
    sleep_for(milliseconds(BALANCE_CAR_TASK_INIT_TIME_MS));

    while (true)
    {
        car.feedback_update(); // 读取传感器
        // set_control 内部已经加了锁，或者是这里加锁：
        // { std::lock_guard<std::mutex> lock(xGlobalDataMutex); car.set_control(); }
        car.set_control();     
        car.solve();           // 计算 PID
        car.output();          // 输出电机
        sleep_for(milliseconds(BALANCE_CAR_CONTROL_TIME_MS));
    }
}



int main()
{   
    


    // 1. 初始化 CAN
    if (!can_receive.init("can0")) {
        return -1;
    }

    thread receiver(receive_thread_func);
    receiver.detach(); 

    // 创建并启动线程
    //t_comm(communicate_Task);
    thread t_balance(balance_Task);


    //t_comm.join();
    t_balance.join();

    return 0;
}