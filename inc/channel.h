/*
 * channel.h
 */

#ifndef INC_CHANNEL_H_
#define INC_CHANNEL_H_
#include "chip.h"

#define CCAN_BAUD_RATE 500000 // 500Kbps

volatile uint32_t *CAN_CNTL = 0x40050000;
volatile uint32_t *CAN_TEST = 0x40050014;

void Setup_Channel();
void Send_XL_Data(int32_t);
void Send_G_Data(int32_t);
void Send_M_Data(int32_t);

#define XL_DATA_CAN_MODE 0x101
#define G_DATA_CAN_MODE  0x102
#define M_DATA_CAN_MODE  0x103


#endif /* INC_CHANNEL_H_ */
