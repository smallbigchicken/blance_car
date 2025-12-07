#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "fdcan.h"
#include "ws2812.h"
#include "cmsis_os.h"
#include "bsp_fdcan.h"
#include "struct_typedef.h"

//DM设备和DJI设备CAN总线分离
#define LEGS_CAN hfdcan1
#define IMU_CAN hfdcan3





typedef enum
{
  CAN_LEFT_LEG_MOTOR_ID = 0x201, 
  CAN_RIGHT_LEG_MOTOR_ID = 0x202, 
  CAN_LEGS_ALL_ID = 0x200,

  DM_IMU_CAN_ID = 0x01,
  DM_IMU_MASTER_ID = 0x11,
  DM_IMU_REQ_ID = 0x6FF

} can_msg_id_e;


typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} dji_motor_measure_t;


#define PITCH_CAN_MAX   (90.0f)
#define PITCH_CAN_MIN   (-90.0f)
#define ROLL_CAN_MAX    (180.0f)
#define ROLL_CAN_MIN    (-180.0f)
#define YAW_CAN_MAX     (180.0f)
#define YAW_CAN_MIN     (-180.0f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)

typedef struct
{
  int pitch, roll, yaw;
  int x_gyro,y_gyro,z_gyro;
} dm_imu_measure_t;


class Can_receive{
public:


//云台控制机构电机反馈数据结构体
dji_motor_measure_t* legs[2];
dm_imu_measure_t* imu;

//发送数组
uint8_t legs_send_data[8];
uint8_t imu_send_data[4];


//初始化CAN通信
void init();

//初始化
const dji_motor_measure_t *get_dji_motor_measure_point(uint8_t i);
const dm_imu_measure_t *get_dm_imu_measure_point();

//回调
void get_dji_motor_measure(dji_motor_measure_t *dji_motor, uint8_t data[8]);
void get_dm_imu_measure(dm_imu_measure_t *imu, uint8_t data[8]);

//发送
void can_cmd_leg_motor(int16_t left_leg, int16_t right_leg,uint16_t ID);
void can_cmd_imu_request(uint16_t ID);




int float_to_uint(fp32 x_float, fp32 x_min, fp32 x_max, int bits);
fp32 uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits);




};


extern Can_receive can_receive;

#endif
