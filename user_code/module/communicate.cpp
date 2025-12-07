#include "communicate.h"

int i=0;

Communicate communicate;
Imu imu(can_receive.get_dm_imu_measure_point());


void Communicate::init()
{
    can_receive.init();
}

void Communicate::run()
{
	can_receive.can_cmd_imu_request(DM_IMU_CAN_ID);
	imu.update();
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
		if(hfdcan == &hfdcan3)
		{
			fdcan3_rx_callback();
		}
	}
}




uint8_t rx_data1[8] = {0};
uint16_t id1;
void fdcan1_rx_callback(void)
{
	id1=fdcanx_receive(&hfdcan1, rx_data1);
	switch (id1)
	{
	case CAN_LEFT_LEG_MOTOR_ID:
		can_receive.get_dji_motor_measure(can_receive.legs[0],rx_data1);
		//WS2812_Ctrl(0,0,0);
		break;
	case CAN_RIGHT_LEG_MOTOR_ID:
		can_receive.get_dji_motor_measure(can_receive.legs[1],rx_data1);
		break;
	default:
		break;
	}
	
}
uint8_t rx_data2[8] = {0};
uint16_t id2;
void fdcan2_rx_callback(void)
{
	id2=fdcanx_receive(&hfdcan2, rx_data2);
	switch (id2)
	{
	default:
		break;
	}
	
}
uint8_t rx_data3[8] = {0};
uint16_t id3;
void fdcan3_rx_callback(void)
{
	id3=fdcanx_receive(&hfdcan3, rx_data3);
	switch (id3)
	{
	case DM_IMU_MASTER_ID:
		can_receive.get_dm_imu_measure(can_receive.imu,rx_data3);
		break;
	
	default:
		break;
	}
	
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	

}

