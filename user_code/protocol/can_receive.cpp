#include "can_receive.h"

Can_receive can_receive;



void Can_receive::init()
{
    can_bsp_init();
}

void Can_receive::
can_cmd_leg_motor(int16_t left_leg, int16_t right_leg,uint16_t ID)
{

    legs_send_data[0] = left_leg >> 8;
    legs_send_data[1] = left_leg;
    legs_send_data[2] = right_leg >> 8;
    legs_send_data[3] = right_leg;
    legs_send_data[4] = 0;
    legs_send_data[5] = 0;
    legs_send_data[6] = 0;
    legs_send_data[7] = 0;

    fdcanx_send_data(&LEGS_CAN, ID, legs_send_data,sizeof(legs_send_data));
}



void Can_receive::can_cmd_imu_request_euler(uint16_t ID)
{

    imu_send_data[0] = ID;
    imu_send_data[1] = ID>>8;
    imu_send_data[2] = 0x03;
    imu_send_data[3] = 0xCC;
    fdcanx_send_data(&IMU_CAN, DM_IMU_REQ_ID, imu_send_data,sizeof(imu_send_data));
}

void Can_receive::can_cmd_imu_request_gyro(uint16_t ID)
{

    imu_send_data[0] = ID;
    imu_send_data[1] = ID>>8;
    imu_send_data[2] = 0x02;
    imu_send_data[3] = 0xCC;
    fdcanx_send_data(&IMU_CAN, DM_IMU_REQ_ID, imu_send_data,sizeof(imu_send_data));
}


void Can_receive::get_dji_motor_measure(dji_motor_measure_t *dji_motor, uint8_t data[8])
{
    dji_motor->last_ecd = dji_motor->ecd;
    dji_motor->ecd = (uint16_t)(data[0] << 8 | data[1]);
    dji_motor->speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    dji_motor->given_current = (uint16_t)(data[4] << 8 | data[5]);
    dji_motor->temperate = data[6];
}

void Can_receive::get_dm_imu_measure(dm_imu_measure_t * imu, uint8_t data[8])
{
    switch (data[0])
    {
    case 2:
        imu->x_gyro=uint_to_float(int(data[2]|(data[3]<<8)),GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	    imu->y_gyro=uint_to_float(int(data[4]|(data[5]<<8)),GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	    imu->z_gyro=uint_to_float(int(data[6]|(data[7]<<8)),GYRO_CAN_MIN,GYRO_CAN_MAX,16);
        break;
    case 3:
        imu->pitch=uint_to_float(int(data[3]<<8)|(data[2]),PITCH_CAN_MIN,PITCH_CAN_MAX,16);
        imu->yaw=uint_to_float(int(data[4]|(data[5]<<8)),YAW_CAN_MIN,YAW_CAN_MAX,16);
        imu->roll=uint_to_float(int(data[6]|(data[7]<<8)),ROLL_CAN_MIN,ROLL_CAN_MAX,16);
        break;
    default:
        break;
    }


    
}






int Can_receive::float_to_uint(fp32 x_float, fp32 x_min, fp32 x_max, int bits)
{
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return (int) ((x_float-offset)*((fp32)((1<<bits)-1))/span);
}

fp32 Can_receive::uint_to_float(int x_int, fp32 x_min, fp32 x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return ((fp32)x_int)*span/((fp32)((1<<bits)-1)) + offset;
}





const dji_motor_measure_t *Can_receive::get_dji_motor_measure_point(uint8_t i)
{
   return &legs[i];
}

const dm_imu_measure_t *Can_receive::get_dm_imu_measure_point()
{
   return &imu;
}

