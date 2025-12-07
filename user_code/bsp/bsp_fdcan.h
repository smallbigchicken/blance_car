#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H
#include "main.h"
#include "fdcan.h"
#include "ws2812.h"
void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint16_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);
uint32_t get_fdcan_dlc(uint32_t len);
__weak void fdcan1_rx_callback(void);
__weak void fdcan2_rx_callback(void);
__weak void fdcan3_rx_callback(void);

#endif 

