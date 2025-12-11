#include "can_receive.h"
#include <cstdio>

Can_receive can_receive;

Can_receive::Can_receive() : serial_fd(-1) {}

Can_receive::~Can_receive() {
    if (serial_fd >= 0) {
        // 退出前尝试关闭 CAN 通道
        send_slcan_cmd("C");
        close(serial_fd);
    }
}

// 辅助：发送简单的配置指令
void Can_receive::send_slcan_cmd(const char* cmd) {
    if (serial_fd < 0) return;
    std::string command = std::string(cmd) + "\r";
    write(serial_fd, command.c_str(), command.length());
    usleep(50000); // 稍微延时，给模块处理时间
}

// 配置串口参数
bool Can_receive::configure_serial() {
    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("Error from tcgetattr");
        return false;
    }

    cfsetospeed(&tty, B115200); // USB虚拟串口通常忽略这个，但设上无妨
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; 
    tty.c_iflag &= ~IGNBRK;                     
    tty.c_lflag = 0;                            
    tty.c_oflag = 0;                            
    tty.c_cc[VMIN]  = 0;                        
    tty.c_cc[VTIME] = 1; // 非阻塞读取，100ms 超时

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     
    tty.c_cflag |= (CLOCAL | CREAD);            
    tty.c_cflag &= ~(PARENB | PARODD);          
    tty.c_cflag &= ~CSTOPB;                     
    tty.c_cflag &= ~CRTSCTS;                    

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return false;
    }
    return true;
}

// 初始化函数
bool Can_receive::init(const char* port_name) {
    // 1. 打开串口
    serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        std::cerr << "Error opening " << port_name << ": " << strerror(errno) << std::endl;
        return false;
    }

    // 2. 配置串口
    if (!configure_serial()) {
        close(serial_fd);
        return false;
    }

    std::cout << "Serial Port " << port_name << " Opened. Initializing SLCAN..." << std::endl;

    // 3. SLCAN 初始化序列
    // C = Close (先关闭以防万一)
    send_slcan_cmd("C");
    // S8 = Set bitrate 1M (S6=500k, S8=1M) -> 根据你的电机波特率修改这里！
    send_slcan_cmd("S8"); 
    // O = Open (打开 CAN 通道)
    send_slcan_cmd("O");

    std::cout << "SLCAN Initialized (Mode: 1M)." << std::endl;
    return true;
}

// 发送指令 (重写为 SLCAN 字符串格式)
void Can_receive::can_cmd_leg_motor(int16_t left_leg, int16_t right_leg, uint16_t ID) {
    if (serial_fd < 0) return;

    // 准备数据
    uint8_t d0 = (left_leg >> 8) & 0xFF;
    uint8_t d1 = left_leg & 0xFF;
    uint8_t d2 = (right_leg >> 8) & 0xFF;
    uint8_t d3 = right_leg & 0xFF;

    // 拼装字符串: tIIILDD... (t + ID + 长度 + 数据)
    // 格式: t 200 8 XX XX XX XX 00 00 00 00 \r
    char cmd_buf[64];
    sprintf(cmd_buf, "t%03X8%02X%02X%02X%02X00000000\r", 
            ID, d0, d1, d2, d3);

    // 写入串口
    int nbytes = write(serial_fd, cmd_buf, strlen(cmd_buf));
    if (nbytes < 0) {
        perror("Serial Write Error");
    }
}

// 辅助：解析 Hex 字符
uint8_t Can_receive::hex_char_to_byte(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

// 辅助：解析 "FF" 这种字符串为 255
uint8_t Can_receive::hex_str_to_byte(const char* str) {
    return (hex_char_to_byte(str[0]) << 4) | hex_char_to_byte(str[1]);
}

// 接收函数 (重写为字符串解析)
void Can_receive::receive_once() {
    if (serial_fd < 0) return;

    char buf[128];
    memset(buf, 0, sizeof(buf));

    // 读取串口数据
    int n = read(serial_fd, buf, sizeof(buf) - 1);
    
    if (n > 0) {
        buf[n] = '\0'; // 确保字符串结尾
        
        // --- [调试打印 1]：打印原始串口数据 ---
        // 注意：SLCAN 数据通常以 \r 结尾，打印出来可能会换行
        //std::cout << "[Raw Recv]: " << buf << std::endl; 

        std::string recv_str(buf);
        // 查找标准帧头 't'
        size_t t_pos = recv_str.find('t');
        
        // 确保找到了 't' 且长度足够
        if (t_pos != std::string::npos && recv_str.length() >= t_pos + 21) {
            
            // 1. 解析 ID
            std::string id_str = recv_str.substr(t_pos + 1, 3);
            uint16_t can_id = std::stoi(id_str, nullptr, 16);

            // 2. 解析数据
            uint8_t data[8] = {0};
            // 确保字符串长度足够容纳数据 (t + 3位ID + 1位长度 + 16位数据)
            // 这里的判断逻辑稍微放宽一点，防止越界
            if (recv_str.length() >= t_pos + 21) {
                const char* data_ptr = recv_str.c_str() + t_pos + 5;
                for (int i = 0; i < 8; i++) {
                    data[i] = hex_str_to_byte(data_ptr + i * 2);
                }

                // --- [调试打印 2]：打印解析后的 ID 和 Hex 数据 ---
                // printf(" -> Parsed ID: 0x%03X | Data: ", can_id);
                // for(int i=0; i<8; i++) {
                //     printf("%02X ", data[i]);
                // }
                // printf("\n");
                // ---------------------------------------------

                // 4. 根据 ID 分发数据
                if (can_id == CAN_LEFT_LEG_MOTOR_ID) {
                    get_dji_motor_measure(&legs[0], data);
                }
                else if (can_id == CAN_RIGHT_LEG_MOTOR_ID) {
                    get_dji_motor_measure(&legs[1], data);
                }
            }
        } else {
            // 如果收到了数据但格式不对 (比如没找到 t)，打印个警告
            // std::cout << "[Warning] Invalid format or incomplete frame" << std::endl;
        }
    }
}

// 数据解析回调 (保持原来的逻辑完全不变)
void Can_receive::get_dji_motor_measure(dji_motor_measure_t *dji_motor, uint8_t data[8])
{
    dji_motor->last_ecd = dji_motor->ecd;
    dji_motor->ecd = (uint16_t)(data[0] << 8 | data[1]);
    dji_motor->speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    dji_motor->given_current = (uint16_t)(data[4] << 8 | data[5]);
    dji_motor->temperate = data[6];
}

const dji_motor_measure_t *Can_receive::get_dji_motor_measure_point(uint8_t i)
{
   return &legs[i];
}