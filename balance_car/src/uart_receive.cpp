#include "uart_receive.h"
#include <cstring> // memcpy
#include <cmath>

// 全局实例
Uart_receive uart_receive;

Uart_receive::Uart_receive() : serial_fd(-1) {
    // 初始化数据为 0
    memset(&imu_data, 0, sizeof(imu_data));
}

Uart_receive::~Uart_receive() {
    if (serial_fd != -1) {
        close(serial_fd);
    }
}

bool Uart_receive::init(const char* port_name) {
    // 1. 打开串口
    serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        perror("[IMU] Open Serial Port Failed");
        return false;
    }

    // 2. 配置串口参数
    if (!configure_serial()) {
        close(serial_fd);
        return false;
    }
    
    printf("[IMU] Serial Port %s Initialized Success!\n", port_name);
    return true;
}

const dm_imu_measure_t *Uart_receive::get_imu_measure_point() {
    return &imu_data;
}

bool Uart_receive::configure_serial() {
    struct termios options;
    if (tcgetattr(serial_fd, &options) != 0) {
        perror("[IMU] tcgetattr Failed");
        return false;
    }

    // 设置波特率 115200
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // 8N1 模式
    options.c_cflag &= ~PARENB; // 无校验
    options.c_cflag &= ~CSTOPB; // 1停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8数据位
    
    // 启用接收，忽略 Modem 控制线
    options.c_cflag |= (CLOCAL | CREAD);

    // Raw 模式 (这非常重要！也就是你原来代码里做的那些 &= ~)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭流控
    options.c_oflag &= ~OPOST;                  // 原始输出

    // 设置为非阻塞读取的超时特性 (配合主循环非阻塞)
    // 或者设置为阻塞读取，看你的整个程序架构
    // 这里设置为：读取立即返回
    fcntl(serial_fd, F_SETFL, 0); // 设为阻塞模式，但在 receive_once 里我们会控制
    
    // 应用设置
    if (tcsetattr(serial_fd, TCSANOW, &options) != 0) {
        perror("[IMU] tcsetattr Failed");
        return false;
    }
    return true;
}

void Uart_receive::receive_once() {
    if (serial_fd < 0) return;

    uint8_t temp_buf[64];
    // 非阻塞读取尝试，或者读取当前缓冲区有的所有数据
    int n = read(serial_fd, temp_buf, sizeof(temp_buf));
    
    if (n > 0) {
        // 1. 将新数据追加到成员变量 rx_buffer 中
        rx_buffer.insert(rx_buffer.end(), temp_buf, temp_buf + n);

        // 2. 循环检查缓冲区是否有完整的一包 (长度 19)
        // 协议：Header(55 AA) ... Tail(0A) Length=19
        while (rx_buffer.size() >= 19) {
            
            // 检查帧头 0x55 0xAA
            if (rx_buffer[0] == 0x55 && rx_buffer[1] == 0xAA) {
                
                // 检查帧尾 0x0A (在索引 18 处)
                if (rx_buffer[18] == 0x0A) {
                    // === 找到完整包，进行解析 ===
                    parse_frame(rx_buffer.data());

                    // 从缓存中移除这 19 个字节
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 19);
                } else {
                    // 头对上了，但尾不对，说明可能是假头或者数据错位
                    // 移除第一个字节，继续找下一个 0x55
                    rx_buffer.erase(rx_buffer.begin());
                }
            } else {
                // 没找到头，移除第一个字节，向后滑动
                rx_buffer.erase(rx_buffer.begin());
            }
        }
    }
}

void Uart_receive::parse_frame(const uint8_t* frame) {
    // frame[0]=0x55, frame[1]=0xAA
    uint8_t type = frame[3]; // 功能字

    // 定义联合体用于 Byte -> Float 转换
    union {
        float f;
        uint8_t b[4];
    } x, y, z;

    // 拷贝数据 (小端模式：frame[4]是低字节)
    memcpy(x.b, &frame[4], 4);
    memcpy(y.b, &frame[8], 4);
    memcpy(z.b, &frame[12], 4);

    // 根据类型填充 imu_data 结构体
    // 注意：这里的 type ID (0x02, 0x03) 需要根据你的实际模块确认
    // 假设：0x02 是角速度，0x03 是欧拉角(或加速度)
    
    switch (type) {
        case 0x02: // 角速度
            imu_data.gyro_x = x.f;
            imu_data.gyro_y = y.f;
            imu_data.gyro_z = z.f;
            break;
            
        case 0x03: // 假设这是欧拉角 (如果是加速度，请自行修改变量名)
        case 0x01: // 有些模块欧拉角是 0x01
            imu_data.roll = x.f;
            imu_data.pitch = y.f;
            imu_data.yaw = z.f;
            break;
            
        default:
            break;
    }
}