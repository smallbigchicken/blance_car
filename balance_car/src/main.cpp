#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>
#include "car.h"
#include "GestureReceiver.h"
#include "LCD_gui.h"
#include "pca9557.h"
using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

// ================= 配置参数 =================
#define COMMUNICATE_TASK_INIT_TIME 30
#define COMMUNICATE_CONTROL_TIME_MS 1
#define BALANCE_CAR_TASK_INIT_TIME_MS 200
#define BALANCE_CAR_CONTROL_TIME_MS 1
#define PROGRAM_RUN_TIME_SECONDS 1200 // 主程序运行总时长
// Python 环境和脚本路径配置
const std::string PYTHON_BIN = "/usr/local/miniconda3/bin/python";
const std::string SCRIPT_PATH = "/home/HwHiAiUser/blance_car/balance_car/py/named_pipes.py";
const std::string PIPE_PATH = "/tmp/my_pipe";

// 电机pid参数
float LEG_SPEED_PID[6] = {800.0f, 0.8f, 110.0f, 0.0f, 200.0f, 6000.0f};

// car类初始化
Car car(can_receive.get_dji_motor_measure_point(0),
        can_receive.get_dji_motor_measure_point(1),
        LEG_SPEED_PID);

GestureReceiver gesture_receiver(PIPE_PATH, PYTHON_BIN, SCRIPT_PATH);

// ================= 全局变量 =================
mutex xGlobalDataMutex;

std::atomic<bool> g_enable_balance_loop(true);

std::atomic<bool> g_balance_task_finished(false);

// 小车运动相关
float g_speed_set = 0.0f;    // 线速度 m/s
float g_yaw_rate_set = 0.0f; // 角速度 rad/s
int cmd_gesture_id = -1;

// ================= 任务 1: 通信任务 =================
void communicate_Task()
{
    sleep_for(milliseconds(COMMUNICATE_TASK_INIT_TIME));
    std::cout << "[Comm] 通信初始化开始" << std::endl;

    if (!can_receive.init("/dev/ttyACM0"))
    {
        return;
    }
    std::cout << "[Comm] can通信初始化完成" << std::endl;

    while (!g_balance_task_finished)
    {
        can_receive.receive_once(); // 读取电机反馈报文
        sleep_for(milliseconds(COMMUNICATE_CONTROL_TIME_MS));
    }

    std::cout << "[Comm] 收到平衡任务结束信号，通信任务停止。" << std::endl;
}

// ================= 任务 2: 控制任务 =================
void balance_Task()
{
    sleep_for(milliseconds(BALANCE_CAR_TASK_INIT_TIME_MS));

    // 小车运动控制流程
    while (g_enable_balance_loop)
    {
        car.feedback_update();
        car.set_control(g_speed_set, g_yaw_rate_set); // m/s；rad/s,顺时针为正
        car.solve();
        car.output();
        sleep_for(milliseconds(BALANCE_CAR_CONTROL_TIME_MS));
    }

    car.finish(); // 【
    sleep_for(milliseconds(100));
    std::cout << "[Balance] 电机已停止输出。" << std::endl;

    g_balance_task_finished = true;

    std::cout << "[Balance] 平衡控制任务已退出。" << std::endl;
}

// ================= 任务 3: 视觉识别 =================
void vision_Task()
{
    g_speed_set = 0.0f;    // 线速度 m/s
    g_yaw_rate_set = 0.0f; // 角速度 rad/s
    cmd_gesture_id = gesture_receiver.getGestureId();
    bool flag = false; // 没有在执行手势信号
    auto start = high_resolution_clock::now();
    // 手势控制流程
    while (g_enable_balance_loop)
    {
        if (flag && high_resolution_clock::now() - start > seconds(5)) // 超过5秒没有新手势信号，停止运动
        {
            flag = false;
            g_speed_set = 0.0f;    // 线速度 m/s
            g_yaw_rate_set = 0.0f; // 角速度 rad/s
            cmd_gesture_id = -1;
            cout << "[Vision] 手势控制结束，恢复静止状态" << endl;
        }
        if (!flag)
        {
            g_speed_set = 0.0f;    // 线速度 m/s
            g_yaw_rate_set = 0.0f; // 角速度 rad/s
        }
        cmd_gesture_id = gesture_receiver.getGestureId();
        if (cmd_gesture_id != -1 && !flag)
        {
            start = high_resolution_clock::now();
            flag = true;
            switch (cmd_gesture_id)
            {
            case 1:                    // 前进
                g_speed_set = 0.2f;    // 线速度 m/s
                g_yaw_rate_set = 0.0f; // 角速度 rad/s
                break;
            case 2:                    // 左转
                g_speed_set = 0.15f;   // 线速度 m/s
                g_yaw_rate_set = 0.8f; // 角速度 rad/s
                break;
            case 3:                     // 右转
                g_speed_set = 0.15f;    // 线速度 m/s
                g_yaw_rate_set = -0.8f; // 角速度 rad/s
                break;
            case 4:                    // 后退
                g_speed_set = 0.0f;    // 线速度 m/s
                g_yaw_rate_set = 0.0f; // 角速度 rad/s
                break;
            case 5:                    // 自转
                g_speed_set = 0.0f;    // 线速度 m/s
                g_yaw_rate_set = 1.5f; // 角速度 rad/s
                break;
            default:
                g_speed_set = 0.0f;    // 线速度 m/s
                g_yaw_rate_set = 0.0f; // 角速度 rad/s
                break;
            }
            cout << "[Vision] 接收到手势 ID: " << cmd_gesture_id << " 开始执行运动 -线速度:" << g_speed_set << "m/s 角速度:" << g_yaw_rate_set << "rad/s" << endl;
        }
        sleep_for(milliseconds(50)); // 调整读取频率
    }
    g_speed_set = 0.0f;    // 线速度 m/s
    g_yaw_rate_set = 0.0f; // 角速度 rad/s
    cmd_gesture_id = -1;
}

// ================= 任务 4: LCD 显示 =================
void lcd_Task()
{
    LCD_Init(L2R_U2D, 1000);
    LCD_Clear(BLUE);
    GUI_Show();

    while (g_enable_balance_loop)
    {
        sensor(g_speed_set, g_yaw_rate_set);
    }

    LCD_Exit();
}

// ================= 任务 5: 数码管显示 =================
void LEDnums_Task()
{
    // 数码管上显示倒计时
    pca9557_init("/dev/i2c-7");
    pca9557_setnum(0, 0, 0, 0);
    int time_left = PROGRAM_RUN_TIME_SECONDS;
    while (g_enable_balance_loop)
    {
        time_left--;
        int minutes = time_left / 60;
        int seconds = time_left % 60;
        int tens_seconds = seconds / 10;
        int units_seconds = seconds % 10;
        pca9557_setnum(minutes / 10, minutes % 10, tens_seconds, units_seconds);
        sleep_for(milliseconds(1000));
    }
}

void LEDnums_Task2() // 数码管显示静态信息
{
    pca9557_show();
}

// ================= 主程序入口 =================
int main()
{
    std::cout << "主程序启动，将在 " << PROGRAM_RUN_TIME_SECONDS << " 秒后自动结束。" << std::endl;

    thread t_led(LEDnums_Task);
    thread t_led2(LEDnums_Task2);
    // 启动 Python 视觉识别子进程
    // 必须在开启控制循环之前启动，确保管道就绪
    if (!gesture_receiver.start())
    {
        std::cerr << "[Error] 无法启动视觉模块，程序终止！" << std::endl;
        return -1;
    }
    // 稍微给一点时间让 Python 预热加载模型（可选，视Python脚本启动速度而定）
    sleep_for(milliseconds(500));
    thread t_vision(vision_Task);
    std::cout << "[Main] 视觉模块启动完成。" << std::endl;

    // 初始化标识位
    g_enable_balance_loop = true;
    g_balance_task_finished = false;

    thread t_comm(communicate_Task);
    thread t_balance(balance_Task);
    thread t_lcd(lcd_Task);
    sleep_for(seconds(PROGRAM_RUN_TIME_SECONDS));

    // [修改5] Main 只负责通知 Balance 任务停下来
    std::cout << "[Main] 时间到，请求停止平衡任务..." << std::endl;
    g_enable_balance_loop = false;

    if (t_balance.joinable())
        t_balance.join(); // 建议先 join 平衡任务（逻辑上它先结束）
    if (t_comm.joinable())
        t_comm.join(); // 再 join 通信任务
    if (t_lcd.joinable())
        t_lcd.join(); // 再 join 显示任务
    if (t_vision.joinable())
        t_vision.join(); // 最后 join 视觉任务
    if (t_led.joinable())
        t_led.join();
    if (t_led2.joinable())
        t_led2.join();

    std::cout << "所有线程已退出，程序结束。" << std::endl;
    return 0;
}
