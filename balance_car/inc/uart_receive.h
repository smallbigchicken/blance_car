#ifndef UART_RECEIVE_H
#define UART_RECEIVE_H

#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


typedef struct
{
    float roll;      // 横滚角 (deg)
    float pitch;     // 俯仰角 (deg)
    float yaw;       // 偏航角 (deg)
    
    float gyro_x;    // X轴角速度 (deg/s or rad/s)
    float gyro_y;    // Y轴角速度
    float gyro_z;    // Z轴角速度

} dm_imu_measure_t;

class Uart_receive {
public:
    int serial_fd;              // 串口文件描述符
    dm_imu_measure_t imu_data;  // 存储最新的 IMU 数据

    Uart_receive();
    ~Uart_receive();

    // 初始化：传入设备路径，如 "/dev/ttyACM0"
    bool init(const char* port_name);

    // 获取数据指针 (方便外部直接访问)
    const dm_imu_measure_t *get_imu_measure_point();

    // 接收一次 (读取串口 -> 存入缓存 -> 解析完整包)
    void receive_once();

private:
    // 内部接收缓存 (处理粘包/断包的关键)
    std::vector<uint8_t> rx_buffer;

    // 内部辅助：配置串口
    bool configure_serial();

    // 内部解析函数：处理一帧 19 字节的数据
    void parse_frame(const uint8_t* frame);
};

// 声明外部单例 (如果你想像 Can_receive 那样全局使用)
extern Uart_receive Uart_receive;

#endif