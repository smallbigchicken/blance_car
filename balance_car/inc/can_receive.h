#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include <iostream>
#include <cstring>
#include <cstdint>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// 定义 ID 枚举
typedef enum
{
    CAN_LEFT_LEG_MOTOR_ID = 0x201, 
    CAN_RIGHT_LEG_MOTOR_ID = 0x202, 
    CAN_LEGS_ALL_ID = 0x200,
} can_msg_id_e;

// 定义 DJI 电机数据结构
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} dji_motor_measure_t;

class Can_receive {
public:
    int serial_fd; // 替换原来的 sock_fd
    dji_motor_measure_t legs[2];

    Can_receive();
    ~Can_receive();

    // 初始化：传入设备路径，如 "/dev/ttyACM0"
    bool init(const char* port_name);

    // 获取电机数据指针
    const dji_motor_measure_t *get_dji_motor_measure_point(uint8_t i);

    // 发送给电机 (生成 SLCAN 字符串并写入串口)
    void can_cmd_leg_motor(int16_t left_leg, int16_t right_leg, uint16_t ID);

    // 接收一次 (读取串口并解析字符串)
    void receive_once();

private:
    // 内部数据解析回调 (保持不变)
    void get_dji_motor_measure(dji_motor_measure_t *dji_motor, uint8_t data[8]);

    // 内部辅助：配置串口
    bool configure_serial();
    
    // 内部辅助：发送简短的 SLCAN 指令 (如 "S8", "O")
    void send_slcan_cmd(const char* cmd);
    
    // 内部辅助：16进制字符转数字
    uint8_t hex_char_to_byte(char c);
    uint8_t hex_str_to_byte(const char* str);
};

extern Can_receive can_receive;

#endif