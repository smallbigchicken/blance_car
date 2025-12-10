#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include <iostream>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

//DM设备和DJI设备CAN总线分离
#define LEGS_CAN hfdcan1



typedef enum
{
  CAN_LEFT_LEG_MOTOR_ID = 0x201, 
  CAN_RIGHT_LEG_MOTOR_ID = 0x202, 
  CAN_LEGS_ALL_ID = 0x200,
} can_msg_id_e;


typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} dji_motor_measure_t;




class Can_receive{
public:

int sock_fd;

dji_motor_measure_t legs[2];

uint8_t legs_send_data[8];

Can_receive(){};
~Can_receive();

bool init(const char* ifname);

//初始化
const dji_motor_measure_t *get_dji_motor_measure_point(uint8_t i);


//回调
void get_dji_motor_measure(dji_motor_measure_t *dji_motor, uint8_t data[8]);


//发送
void can_cmd_leg_motor(int16_t left_leg, int16_t right_leg,uint16_t ID);
void receive_once();



};


extern Can_receive can_receive;

#endif
