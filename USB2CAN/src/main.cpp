#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // 1. 创建 Socket
    // PF_CAN = CAN 协议族, SOCK_RAW = 原始套接字
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket creation failed");
        return 1;
    }

    // 2. 指定 CAN 接口 (这里是 can0)
    std::strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr); // 获取接口索引

    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 3. 绑定 Socket 到 can0
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        return 1;
    }

    std::cout << "CAN Interface Initialized. Preparing to send command..." << std::endl;

    // 4. 准备要发送的数据帧 (Frame)
    struct can_frame frame;
    
    
    frame.can_id = 0x200;  
    frame.can_dlc = 8;     
    
   
    frame.data[0] = 0x03; 
    frame.data[1] = 0x00;
    frame.data[2] = 0x03;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    
    while(1){
        // 5. 发送数据
        int nbytes = write(s, &frame, sizeof(struct can_frame));
        
        if (nbytes != sizeof(struct can_frame)) {
            perror("Write error");
            return 1;
        }

        std::cout << "Message sent! ID: 0x" << std::hex << frame.can_id << std::dec << std::endl;

        
        // 这里做一个简单的阻塞接收测试
        struct can_frame frame_read;
        int nbytes_read = read(s, &frame_read, sizeof(struct can_frame));

        if (nbytes_read > 0) {
            std::cout << "Received response from ID: 0x" << std::hex << frame_read.can_id << std::endl;
            std::cout << "Data: ";
            for(int i=0; i < frame_read.can_dlc; i++) {
                printf("%02X ", frame_read.data[i]);
            }
            std::cout << std::endl;
        }
        usleep(5000);
    }
    // 7. 关闭 Socket
    close(s);

    return 0;
}