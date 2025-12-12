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
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);

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
// 辅助函数：确保一定读够 len 个字节
// 因为 read(fd, buf, 17) 并不保证一次就能返回 17 个字节，可能只返回 5 个
// 所以必须用循环确保读够，否则数据会错乱
int read_strictly(int fd, uint8_t* buf, int len) {
    int total_read = 0;
    while (total_read < len) {
        int n = read(fd, buf + total_read, len - total_read);
        if (n < 0) return -1; // 错误
        if (n == 0) continue; // 暂时没数据，继续等
        total_read += n;
    }
    return total_read;
}



void Uart_receive::receive_once() {
    if (serial_fd < 0) return;

    uint8_t header_byte;

    int n = read(serial_fd, &header_byte, 1);
    
    if (n <= 0 || header_byte != 0x55) {
        return; 
    }

    if (read_strictly(serial_fd, &header_byte, 1) < 0) return;

    if (header_byte != 0xAA) {
        return; 
    }

    // ---------------------------------------------------------

    uint8_t body[17];
    if (read_strictly(serial_fd, body, 17) == 17) {
        
        // -----------------------------------------------------
        // 第四步：根据功能字 (Type) 进行分支处理
        // -----------------------------------------------------
        // 对应关系：
        // 完整包: [55] [AA] [ID] [Type] [Data...]
        // body[]:           [0]  [1]    [2...]
        // 所以 body[1] 就是说明书里的 DATA[3] (寄存器ID)
        
        uint8_t data_type = body[1];

        // 临时变量用于拷贝数据
        union { float f; uint8_t b[4]; } v1, v2, v3;

        // body[2]~body[5]   -> Data1
        // body[6]~body[9]   -> Data2
        // body[10]~body[13] -> Data3
        memcpy(v1.b, &body[2], 4);
        memcpy(v2.b, &body[6], 4);
        memcpy(v3.b, &body[10], 4);

        // 分支结构
        if (data_type == 0x02) {
            // === 处理角速度 (Gyro) ===
            imu_data.gyro_x = v1.f;
            imu_data.gyro_y = v2.f;
            imu_data.gyro_z = v3.f;
            // std::cout << "收到角速度包" << std::endl;

        } else if (data_type == 0x03) {
            // === 处理欧拉角 (Angle) ===
            imu_data.roll  = (v1.f*2*PI/360);
            imu_data.pitch = (v2.f*2*PI/360);
            imu_data.yaw   = (v3.f*2*PI/360);
            
            // std::cout << "roll：" <<imu_data.roll<< std::endl;
            // std::cout << "pitch：" <<imu_data.pitch<< std::endl;
            // std::cout << "yaw：" <<imu_data.yaw<< std::endl;
        }
        

    }
}

