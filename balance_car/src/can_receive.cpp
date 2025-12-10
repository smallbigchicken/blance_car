#include "can_receive.h"



Can_receive can_receive;

bool Can_receive::init(const char* ifname) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    // 1. 创建 Socket
    if ((sock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket creation failed");
        return false;
    }

    // 2. 指定 CAN 接口
    std::strcpy(ifr.ifr_name, ifname);
    if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("IOCTL failed");
        return false;
    }

    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 3. 绑定
    if (bind(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        return false;
    }

    std::cout << "Linux CAN Interface " << ifname << " Initialized." << std::endl;
    return true;
}

// 发送指令
void Can_receive::can_cmd_leg_motor(int16_t left_leg, int16_t right_leg, uint16_t ID) {
    struct can_frame frame;

    frame.can_id = ID; 
    frame.can_dlc = 8;
    
    frame.data[0] = (left_leg >> 8);
    frame.data[1] = left_leg;
    
    frame.data[2] = (right_leg >> 8);
    frame.data[3] = right_leg;
    
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    // 发送
    if (sock_fd >= 0) {
        int nbytes = write(sock_fd, &frame, sizeof(struct can_frame));
        if (nbytes != sizeof(struct can_frame)) {
            perror("CAN Write Error");
        }
    }
}



void Can_receive::receive_once() {
    struct can_frame frame_read;
    
    int nbytes = read(sock_fd, &frame_read, sizeof(struct can_frame));

    if (nbytes > 0) {
        // 根据 ID 判断是哪个电机
        if (frame_read.can_id == CAN_LEFT_LEG_MOTOR_ID) {
            
            get_dji_motor_measure(&legs[0], frame_read.data);
        }
        else if (frame_read.can_id == CAN_RIGHT_LEG_MOTOR_ID) {
            get_dji_motor_measure(&legs[1], frame_read.data);
        }
    }
}


/**
 * @brief          DJI电机电流接收(云台上为摩擦轮电机和拨弹轮电机)
 * @param[out]     dji_motor: 指向电机数据的指针
 */
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

Can_receive::~Can_receive() {
    if (sock_fd >= 0) {
        close(sock_fd);
    }
}