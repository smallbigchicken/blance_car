#include "communicate.h"

int tickt=0;
Communicate communicate;


void Communicate::init()
{
	dt7.init(DT7_UART,DT7_Rx_Buffer,DT7_RX_BUFFER_SIZE);
    can_receive.init();
}

void Communicate::run()
{	
	if(tickt) can_receive.can_cmd_imu_request_gyro(DM_IMU_CAN_ID);
	else can_receive.can_cmd_imu_request_euler(DM_IMU_CAN_ID);
	tickt=!tickt;
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
		can_receive.get_dji_motor_measure(&can_receive.legs[0],rx_data1);
		//WS2812_Ctrl(0,0,0);
		break;
	case CAN_RIGHT_LEG_MOTOR_ID:
		can_receive.get_dji_motor_measure(&can_receive.legs[1],rx_data1);
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
		can_receive.get_dm_imu_measure(&can_receive.imu,rx_data3);
		break;
	
	default:
		break;
	}
	
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == UART5)
        {
			dt7.unpack();
			HAL_UARTEx_ReceiveToIdle_DMA(huart, dt7.Rx_Buffer , dt7.Rx_Buffer_Size);
        }

}
uint32_t g_UartErrorDebug = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
if(huart->Instance == UART5)
    {
        // ============================================================
        // 1. 【核心诊断步骤】抓取错误码
        // ============================================================
        // 程序卡死后，请在调试器(Debug)的 Watch 窗口查看 g_UartErrorDebug 的值
        g_UartErrorDebug = huart->ErrorCode;

        // 如果你有串口打印功能，也可以取消下面的注释打印出来
        // printf("UART Error Code: 0x%02X\r\n", huart->ErrorCode);


        // ============================================================
        // 2. 暴力清除所有可能的标志位 (宁可错杀，不可放过)
        // ============================================================
        // 停止 DMA，防止后台还在报错
        HAL_UART_DMAStop(huart);

        // 清除 UART 所有的硬件错误标志
        __HAL_UART_CLEAR_OREFLAG(huart); // 0x08: 溢出 (最常见)
        __HAL_UART_CLEAR_NEFLAG(huart);  // 0x04: 噪声 (电机干扰常见)
        __HAL_UART_CLEAR_FEFLAG(huart);  // 0x02: 帧错误 (波特率不对)
        __HAL_UART_CLEAR_PEFLAG(huart);  // 0x01: 校验错误
        
        // 清除 IDLE 标志 (防止空闲中断同时也触发)
        __HAL_UART_CLEAR_IDLEFLAG(huart);


        // ============================================================
        // 3. 【关键】处理 DMA 错误 (0x10)
        // ============================================================
        // 如果 g_UartErrorDebug 是 0x10，说明是 DMA 传输出了问题，而不是串口本身
        // 这种情况下，单纯清串口标志没用，必须重置 DMA 状态
        if (huart->ErrorCode & HAL_UART_ERROR_DMA)
        {
            // HAL库通常会自动处理，但我们需要确保状态复位
            // 可以在这里加一个断点，看看是不是进了这里
        }


        // ============================================================
        // 4. 暴力复活 HAL 库状态机
        // ============================================================
        // 这几行是防止"卡死"的关键。如果 HAL 库认为还在 ERROR 状态，
        // 它是绝对不会让你重新启动接收的。
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;


        HAL_UARTEx_ReceiveToIdle_DMA(huart, dt7.Rx_Buffer, dt7.Rx_Buffer_Size);
	}
}