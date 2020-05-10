/**
 * channel.c
 *
 * Contains all the essential functions to
 * communicate to the bank station through
 * CAN bus.
 */
#include "channel.h"

/**
 * Private definitions
 */

CCAN_MSG_OBJ_T tx_msg_obj;
CCAN_MSG_OBJ_T rx_msg_obj;

// LPCOpen CAN baud rate calculator function
void baudrateCalculate(uint32_t baud_rate, uint32_t *can_api_timing_cfg)
{
	uint32_t pClk, div, quanta, segs, seg1, seg2, clk_per_bit, can_sjw;
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);
	pClk = Chip_Clock_GetMainClockRate();

	clk_per_bit = pClk / baud_rate;

	for (div = 0; div <= 15; div++) {
		for (quanta = 1; quanta <= 32; quanta++) {
			for (segs = 3; segs <= 17; segs++) {
				if (clk_per_bit == (segs * quanta * (div + 1))) {
					segs -= 3;
					seg1 = segs / 2;
					seg2 = segs - seg1;
					can_sjw = seg1 > 3 ? 3 : seg1;
					can_api_timing_cfg[0] = div;
					can_api_timing_cfg[1] =
						((quanta - 1) & 0x3F) | (can_sjw & 0x03) << 6 | (seg1 & 0x0F) << 8 | (seg2 & 0x07) << 12;
					return;
				}
			}
		}
	}
}

void enableLoopbackMode() {
	CAN_CNTL |= 0x80; // enable test mode
	CAN_TEST |= 0x10; // enable loopback mode
}

void send32bitValue(uint32_t mode_id, uint32_t val) {
	tx_msg_obj.mode_id = mode_id;
	tx_msg_obj.dlc = 4;
	tx_msg_obj.data[0] = (uint8_t) val;
	tx_msg_obj.data[1] = (uint8_t)(val >> 8);
	tx_msg_obj.data[2] = (uint8_t)(val >> 16);
	tx_msg_obj.data[3] = (uint8_t)(val >> 24);
	LPC_CCAN_API->can_transmit(&tx_msg_obj);
}

/**
 * @brief	CCAN Interrupt Handler
 * @return	Nothing
 * @note	The CCAN interrupt handler must be provided by the user application.
 *	It's function is to call the isr() API located in the ROM
 */
void CAN_IRQHandler(void) {
	LPC_CCAN_API->isr();
}

void CAN_rx(uint8_t msg_obj_num) {
	/* Determine which CAN message has been received */
	msg_obj.msgobj = msg_obj_num;
	/* Now load up the msg_obj structure with the CAN message */
	LPC_CCAN_API->can_receive(&msg_obj);
	if (msg_obj_num == 1 || msg_obj_num == 2) {
		/* Simply transmit CAN frame (echo) with with ID +0x100 via buffer 2 */
		DEBUGOUT("Received message from %x: %x%x%x%x%x%x%x%x (dlc %u)\n",
				msg_obj.mode_id,
				msg_obj.data[0],
				msg_obj.data[1],
				msg_obj.data[2],
				msg_obj.data[3],
				msg_obj.data[4],
				msg_obj.data[5],
				msg_obj.data[6],
				msg_obj.data[7],
				msg_obj.dlc);

		msg_obj.mode_id += 0x100;
		LPC_CCAN_API->can_transmit(&msg_obj);
	}
}

/*	CAN transmit callback */
void CAN_tx(uint8_t msg_obj_num) {}

/*	CAN error callback */
void CAN_error(uint32_t error_info) {}

void Setup_Channel() {
	uint32_t CanApiClkInitTable[2];

	/* Publish CAN Callback Functions */
	CCAN_CALLBACKS_T callbacks = {
		CAN_rx,
		CAN_tx,
		CAN_error,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
	};

	baudrateCalculate(CCAN_BAUD_RATE, CanApiClkInitTable);

	LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);

	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);

	// setup receiving frames
	rx_msg_obj.msgobj = 1;
	rx_msg_obj.mode_id = 0x0;
	rx_msg_obj.mask = 0xFFFC;
	LPC_CCAN_API->config_rxmsgobj(&rx_msg_obj);

	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);
}

void Send_XL_Data(int32_t val) {
	send32bitValue(XL_DATA_CAN_MODE, (uint32_t)val);
}

void Send_G_Data(int32_t val) {
	send32bitValue(G_DATA_CAN_MODE, (uint32_t)val);
}

void Send_M_Data(int32_t val) {
	send32bitValue(M_DATA_CAN_MODE, (uint32_t)val);
}
