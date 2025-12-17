#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic> 
#include "car.h"

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

// ================= 配置参数 =================
#define COMMUNICATE_TASK_INIT_TIME       30
#define COMMUNICATE_CONTROL_TIME_MS      1
#define BALANCE_CAR_TASK_INIT_TIME_MS    200
#define BALANCE_CAR_CONTROL_TIME_MS      1
#define PROGRAM_RUN_TIME_SECONDS         10  

float LEG_SPEED_PID[6] = {800.0f,0.8f,110.0f ,0.0f,200.0f,6000.0f};

Car car(can_receive.get_dji_motor_measure_point(0),
        can_receive.get_dji_motor_measure_point(1),
        uart_receive.get_imu_measure_point(),
        LEG_SPEED_PID);

// ================= 全局变量 =================
mutex xGlobalDataMutex; 

// [修改1] 标志位改名，逻辑分离
// 标志A：控制主程序是否让“平衡任务”继续运行
std::atomic<bool> g_enable_balance_loop(true); 

// 标志B：平衡任务是否彻底结束（包括刹车过程），用来控制通信任务退出
// 初始为 false，只有当平衡任务彻底跑完后，才会变成 true
std::atomic<bool> g_balance_task_finished(false); 

// ================= 任务 1: 通信任务 =================
void communicate_Task() 
{
    sleep_for(milliseconds(COMMUNICATE_TASK_INIT_TIME));
    std::cout << "[Comm] 通信初始化开始" << std::endl;
    
    if (!can_receive.init("/dev/ttyACM0")) {
        return;
    }
    std::cout << "[Comm] can通信初始化完成" << std::endl;

    // [修改2] 通信任务的生命周期不再由 main 决定，而是看 balance 任务是否完成
    // 只要 balance 任务没彻底结束 (!g_balance_task_finished)，我就得一直发数据
    while (!g_balance_task_finished)
    {
        can_receive.receive_once(); 
        //uart_receive.receive_once();
        sleep_for(milliseconds(COMMUNICATE_CONTROL_TIME_MS));
    }
    
    std::cout << "[Comm] 收到平衡任务结束信号，通信任务停止。" << std::endl;
}

// ================= 任务 2: 平衡控制任务 =================
void balance_Task()
{
    sleep_for(milliseconds(BALANCE_CAR_TASK_INIT_TIME_MS));

    // [修改3] 这里监听 Main 发出的停止信号
    while (g_enable_balance_loop)
    {
        car.feedback_update(); 
        car.set_control(-0.15, 0);     
        car.solve();           
        car.output();          
        sleep_for(milliseconds(BALANCE_CAR_CONTROL_TIME_MS));
    }

    // // --- 进入安全刹车阶段 ---
    // int i = 1000;
    // std::cout << "[Balance] 正在执行安全刹车序列..." << std::endl;
    // while(i--) {
    //     car.feedback_update(); 
    //     car.set_control(0, 0); // 速度设为0
    //     car.solve();        
    //     car.output();       // 【关键】此时 communicate_Task 还在跑，所以这帧数据能发出去！
    //     sleep_for(milliseconds(BALANCE_CAR_CONTROL_TIME_MS));
    // }
    car.finish();       // 【关键】此时 communicate_Task 还在跑，所以这帧数据能发出去！
    sleep_for(milliseconds(100));
    std::cout << "[Balance] 电机已停止输出。" << std::endl;

    // [修改4] 刹车完毕，交出接力棒，通知通信任务可以下班了
    g_balance_task_finished = true; 
    
    std::cout << "[Balance] 平衡控制任务已退出。" << std::endl;
}

int main()
{   
    std::cout << "主程序启动，将在 " << PROGRAM_RUN_TIME_SECONDS << " 秒后自动结束。" << std::endl;

    // 先把标志位重置好（防止意外）
    g_enable_balance_loop = true;
    g_balance_task_finished = false;

    thread t_comm(communicate_Task);
    thread t_balance(balance_Task);

    sleep_for(seconds(PROGRAM_RUN_TIME_SECONDS));

    // [修改5] Main 只负责通知 Balance 任务停下来
    std::cout << "[Main] 时间到，请求停止平衡任务..." << std::endl;
    g_enable_balance_loop = false; 

    // [修改6] 等待线程回收
    // 注意：虽然我们只修改了 g_enable_balance_loop，但 t_balance 会在结束后
    // 自动修改 g_balance_task_finished，从而导致 t_comm 退出。
    // 所以这里依然是安全的。
    if (t_balance.joinable()) t_balance.join(); // 建议先 join 平衡任务（逻辑上它先结束）
    if (t_comm.joinable()) t_comm.join();       // 再 join 通信任务

    std::cout << "所有线程已退出，程序结束。" << std::endl;
    return 0;
}