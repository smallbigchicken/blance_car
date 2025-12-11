// #include <iostream>
// #include <string>
// #include <vector>
// #include <cstdio>
// #include <cstring>
// #include <unistd.h>     // UNIX 标准函数定义
// #include <fcntl.h>      // 文件控制定义
// #include <termios.h>    // POSIX 终端控制定义
// #include <errno.h>

// // 串口配置函数
// int open_serial_port(const char* portname) {
//     int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
//     if (fd < 0) {
//         perror("Error opening serial port");
//         return -1;
//     }

//     struct termios tty;
//     if (tcgetattr(fd, &tty) != 0) {
//         perror("Error from tcgetattr");
//         return -1;
//     }

//     cfsetospeed(&tty, B115200); // USB 虚拟串口通常忽略波特率，但设置一下较好
//     cfsetispeed(&tty, B115200);

//     tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
//     tty.c_iflag &= ~IGNBRK;                     // disable break processing
//     tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing
//     tty.c_oflag = 0;                            // no remapping, no delays
//     tty.c_cc[VMIN]  = 0;                        // read doesn't block
//     tty.c_cc[VTIME] = 5;                        // 0.5 seconds read timeout

//     tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl
//     tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls, enable reading
//     tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
//     tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
//     tty.c_cflag &= ~CRTSCTS;                    // hardware flow control disabled

//     if (tcsetattr(fd, TCSANOW, &tty) != 0) {
//         perror("Error from tcsetattr");
//         return -1;
//     }
//     return fd;
// }

// // 发送指令给 SLCAN 模块
// void send_cmd(int fd, std::string cmd) {
//     cmd += "\r"; // 必须以回车结尾
//     write(fd, cmd.c_str(), cmd.length());
//     usleep(100000); // 稍微等待模块处理
// }

// int main() {
//     const char* portname = "/dev/ttyACM0"; // 你的设备名
//     int fd = open_serial_port(portname);
//     if (fd < 0) return 1;

//     std::cout << "Serial port opened. Initializing SLCAN..." << std::endl;

//     // ---------------------------------------------------------
//     // 1. 初始化 SLCAN 模块 (配置波特率 1M)
//     // ---------------------------------------------------------
//     // C = Close (先关闭以防万一)
//     send_cmd(fd, "C");
//     // S8 = Set bitrate to 1M (S6 = 500k)
//     send_cmd(fd, "S8"); 
//     // O = Open (打开 CAN 通道)
//     send_cmd(fd, "O");

//     std::cout << "SLCAN initialized at 1Mbps. Starting loop..." << std::endl;

//     // ---------------------------------------------------------
//     // 2. 准备发送的数据
//     // ---------------------------------------------------------
//     // 目标: ID 0x200, DLC 8, Data: 05 00 ...
//     // SLCAN 标准帧格式: tIIILDD... (t + 3位ID + 1位长度 + 数据)
//     // 例如: t20080500000000000000
//     char command_str[50];
//     int can_id = 0x200;
    
//     // 拼装字符串
//     sprintf(command_str, "t%03X%d%02X%02X%02X%02X%02X%02X%02X%02X\r",
//             can_id, 8, 
//             0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00);

//     while(1) {
//         // --- 发送 ---
//         int n_written = write(fd, command_str, strlen(command_str));
//         if (n_written > 0) {
//             std::cout << "Sent: " << command_str; // 注意 command_str 里已经有 \r 了
//         }

//         // --- 接收 (非阻塞读取) ---
//         char buf[256];
//         memset(buf, 0, sizeof(buf));
//         int n_read = read(fd, buf, sizeof(buf)-1);
        
//         if (n_read > 0) {
//             // 接收到的数据也是字符串，例如 "t00181122..."
//             // 这里简单打印出来，你可以根据 't' 或 'T' 来解析
//             std::cout << "Received: " << buf << std::endl;
//         }

//         usleep(5000); // 5ms 延时
//     }

//     close(fd);
//     return 0;
// }






#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// ==========================================
// 1. 定义数据结构
// ==========================================

// 用于存储 CAN 帧的结构体
struct CanFrame {
    uint32_t id;             // CAN ID (例如 0x200)
    std::vector<uint8_t> data; // 数据载荷 (最多8字节)
};



enum CanBaudRate {
    CAN_100K = 4,
    CAN_250K = 5,
    CAN_500K = 6,
    CAN_1M   = 8
};

class SlcanDevice {
private:
    int fd;                 // 串口文件描述符
    std::string port_name;  // 端口名
    bool is_connected;

    // 内部函数：配置串口参数 (termios)
    bool configure_port() {
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            perror("Get serial attributes failed");
            return false;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit
        tty.c_iflag &= ~IGNBRK;                     // disable break processing
        tty.c_lflag = 0;                            // no signaling chars, no echo
        tty.c_oflag = 0;                            // no remapping
        tty.c_cc[VMIN]  = 0;                        // read doesn't block
        tty.c_cc[VTIME] = 1;                        // 0.1 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl
        tty.c_cflag |= (CLOCAL | CREAD);            // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
        tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;                    // no flow control

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            perror("Set serial attributes failed");
            return false;
        }
        return true;
    }

    // 内部函数：发送原始字符串指令 (如 "S8", "O")
    void send_raw_cmd(std::string cmd) {
        cmd += "\r"; 
        write(fd, cmd.c_str(), cmd.length());
        usleep(50000); // 给模块一点反应时间
    }

public:
    SlcanDevice(std::string port) : port_name(port), fd(-1), is_connected(false) {}

    ~SlcanDevice() {
        close_device();
    }

    // --- 函数 1: 初始化 ---
    bool init(CanBaudRate baud) {
        fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            std::cerr << "Error: Cannot open " << port_name << std::endl;
            return false;
        }

        if (!configure_port()) {
            close(fd);
            return false;
        }

        std::cout << "[Info] Port opened. Configuring SLCAN..." << std::endl;
        
        // SLCAN 初始化序列
        send_raw_cmd("C"); // Close
        
        // 设置波特率
        std::string speed_cmd = "S" + std::to_string(baud);
        send_raw_cmd(speed_cmd);
        
        send_raw_cmd("O"); // Open

        is_connected = true;
        std::cout << "[Info] SLCAN initialized success." << std::endl;
        return true;
    }

    void close_device() {
        if (is_connected && fd >= 0) {
            send_raw_cmd("C"); // 关闭 CAN 通道
            close(fd);
            fd = -1;
            is_connected = false;
            std::cout << "[Info] Device closed." << std::endl;
        }
    }

    // --- 函数 2: 发送 CAN 帧 ---
    bool send_frame(const CanFrame& frame) {
        if (!is_connected) return false;

        if (frame.data.size() > 8) {
            std::cerr << "[Error] Data too long!" << std::endl;
            return false;
        }

        // 格式化为 SLCAN 字符串: tIIILDD... (例如 t20081122...)
        char cmd_buf[64];
        
        // 这是一个稍微复杂的 sprintf，用来把 data 里的字节拼接到字符串后面
        int len = sprintf(cmd_buf, "t%03X%ld", frame.id, frame.data.size());
        for (uint8_t byte : frame.data) {
            len += sprintf(cmd_buf + len, "%02X", byte);
        }
        sprintf(cmd_buf + len, "\r"); // 结尾加回车

        int n = write(fd, cmd_buf, strlen(cmd_buf));
        if (n > 0) {
            std::cout << "[TX] Sent ID: 0x" << std::hex << frame.id << std::dec << std::endl;
            return true;
        }
        return false;
    }

    // --- 函数 3: 接收 CAN 帧 ---
    // 返回 true 表示收到了有效数据，false 表示没有数据
    bool receive_frame(CanFrame& out_frame) {
        if (!is_connected) return false;

        char buf[256];
        memset(buf, 0, sizeof(buf));
        
        // 尝试读取 (非阻塞或短超时)
        int n = read(fd, buf, sizeof(buf) - 1);
        
        if (n <= 0) return false; // 没有数据

        // 简单的解析逻辑 (假设一次读到一个完整的 't' 开头的数据)
        // 注意：生产环境中需要处理粘包和断包，这里为了演示做了简化
        std::string recv_str(buf);
        if (recv_str.empty() || recv_str[0] != 't') return false; 

        // 解析 tIIILDD...
        // 示例: t00181122334455667788
        try {
            std::string id_str = recv_str.substr(1, 3);
            std::string len_str = recv_str.substr(4, 1);
            
            out_frame.id = std::stoi(id_str, nullptr, 16);
            int dlc = std::stoi(len_str);
            
            out_frame.data.clear();
            for (int i = 0; i < dlc; i++) {
                // 每个字节占 2 个字符
                std::string byte_str = recv_str.substr(5 + i*2, 2);
                out_frame.data.push_back(std::stoi(byte_str, nullptr, 16));
            }
            return true;
        } catch (...) {
            return false; // 解析失败
        }
    }
};

// ==========================================
// 3. 主程序 (Main)
// ==========================================
int main() {
    // 1. 创建对象
    SlcanDevice can_dev("/dev/ttyACM0");

    // 2. 初始化 (1M 波特率)
    if (!can_dev.init(CAN_1M)) {
        return -1;
    }

    // 准备要发送的数据
    CanFrame my_msg;
    my_msg.id = 0x200;
    // 填充数据: 03 00 03 00 00 ...
    my_msg.data = {0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};

    while (true) {
        // --- 发送 ---
        can_dev.send_frame(my_msg);

        // --- 接收 ---
        CanFrame rx_msg;
        if (can_dev.receive_frame(rx_msg)) {
            std::cout << "[RX] ID: 0x" << std::hex << rx_msg.id << " Data: ";
            for (auto b : rx_msg.data) {
                std::cout << std::setw(2) << std::setfill('0') << (int)b << " ";
            }
            std::cout << std::dec << std::endl;
        }

        usleep(5000); // 5ms 循环间隔
    }

    return 0;
}