/*
 * channel.h
 */

#ifndef INC_CHANNEL_H_
#define INC_CHANNEL_H_
#include "chip.h"

#define CCAN_BAUD_RATE 1000000 // 500Kbps

void chSetup();
void chSend32BitValue(uint32_t, uint32_t);
void chSendBuffOverflow();

#define CAN_ADDR_OFFSET 0x101


#endif /* INC_CHANNEL_H_ */
