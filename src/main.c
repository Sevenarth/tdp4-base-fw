/*
 * @brief CCAN on-chip driver example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TEST_CCAN_BAUD_RATE 500000

CCAN_MSG_OBJ_T msg_obj;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
#ifdef DEBUG_ENABLE
static const char menu[] =
	"**************** CAN Demo Menu ****************\r\n"
	"\t0: Exit Demo\r\n"
	"\t1: Send message object\r\n"
	"\t2: Toggle loopback\r\n";
#endif

/* Get an integer input from UART */
static int con_get_input(const char *str)
{
#ifdef DEBUG_ENABLE
	int input_valid = 0;
	int x;
	char ch[16], *ptr;
	int i = 0;

	while (!input_valid) {
		DEBUGOUT("%s", str);
		while (1) {
			x = DEBUGIN();
			if (x == EOF) {
				continue;
			}
			if (i >= sizeof(ch) - 2) {
				break;
			}
			if (((x == '\r') || (x == '\n')) && i) {
				DEBUGOUT("\r\n");
				break;
			}
			if (x == '\b') {
				if (i) {
					DEBUGOUT("\033[1D \033[1D");
					i--;
				}
				continue;
			}
			DEBUGOUT("%c", x);
			ch[i++] = x;
		}
		ch[i] = 0;
		i = strtol(ch, &ptr, 16);
		if (*ptr) {
			i = 0;
			DEBUGOUT("Invalid input. Retry!\r\n");
			continue;
		}
		input_valid = 1;
	}
	return i;
#else
	static int sind = -1;
	static uint8_t val[] = {5, I2C_SLAVE_IOX_ADDR, 1, 0};
	if (sind >= sizeof(val)) {
		sind = -1;
	}
	while (sind < 0 && (tick_cnt & 0x7F)) {}
	if (sind < 0) {
		sind = 0;
		val[3] = !val[3];
		tick_cnt++;
	}
	return val[sind++];
#endif
}

static int can_menu(void)
{
	DEBUGOUT(menu);
	return con_get_input("\r\nSelect an option [0 - 2] :");
}

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

/*	CAN receive callback */
/*	Function is executed by the Callback handler after
    a CAN message has been received */
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

		msg_obj.msgobj = 0;
		msg_obj.mode_id += 0x100;
		LPC_CCAN_API->can_transmit(&msg_obj);
	}
}

/*	CAN transmit callback */
/*	Function is executed by the Callback handler after
    a CAN message has been transmitted */
void CAN_tx(uint8_t msg_obj_num) {
	DEBUGOUT("info: transmitted msg obj num %u\n", msg_obj_num);
}

/*	CAN error callback */
/*	Function is executed by the Callback handler after
    an error has occured on the CAN bus */
void CAN_error(uint32_t error_info) {
	DEBUGOUT("err: %x\n", error_info);
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

/*****************************************************************************
 * Public functions
 ****************************************************************************/

volatile uint32_t *CAN_CNTL = 0x40050000;
volatile uint32_t *CAN_TEST = 0x40050014;

/**
 * @brief	Main routine for CCAN_ROM example
 * @return	Nothing
 */
int main(void)
{
	int xflag = 0;
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
	SystemCoreClockUpdate();
	Board_Init();
	baudrateCalculate(TEST_CCAN_BAUD_RATE, CanApiClkInitTable);

	LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);
	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);
	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);

	/* Configure message object 1 to receive all 11-bit messages 0x300-0x3FF */
	msg_obj.msgobj = 1;
	msg_obj.mode_id = 0x300;
	msg_obj.mask = 0x700;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	/* Configure message object 1 to receive all 11-bit messages 0x400-0x4FF */
	msg_obj.msgobj = 2;
	msg_obj.mode_id = 0x400;
	msg_obj.mask = 0x700;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	/* Configure message object 1 to receive all 11-bit messages 0x500-0x5FF */
	msg_obj.msgobj = 3;
	msg_obj.mode_id = 0x500;
	msg_obj.mask = 0x700;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	/* Send a simple one time CAN message */
	msg_obj.msgobj  = 0;
	msg_obj.mode_id = 0x345;
	msg_obj.mask    = 0x0;
	msg_obj.dlc     = 4;
	msg_obj.data[0] = 'T';	// 0x54
	msg_obj.data[1] = 'E';	// 0x45
	msg_obj.data[2] = 'S';	// 0x53
	msg_obj.data[3] = 'T';	// 0x54
	LPC_CCAN_API->can_transmit(&msg_obj);

	while (!xflag) {
		switch (can_menu()) {
		case 0:
			xflag = 1;
			DEBUGOUT("End of CAN Demo! Bye!\r\n");
			break;

		case 1:
		{
			int mode_id = con_get_input("Enter mode id: ");

			/* Send a simple one time CAN message */
			msg_obj.msgobj  = 0;
			msg_obj.mode_id = mode_id;
			msg_obj.mask    = 0x0;
			msg_obj.dlc     = 4;
			msg_obj.data[0] = 'T';	// 0x54
			msg_obj.data[1] = 'E';	// 0x45
			msg_obj.data[2] = 'S';	// 0x53
			msg_obj.data[3] = 'T';	// 0x54
			LPC_CCAN_API->can_transmit(&msg_obj);

			break;
		}
		case 2:
		{
			*CAN_CNTL ^= 1 << 7;
			*CAN_TEST ^= 1 << 4;
			DEBUGOUT("Loopback toggled! Status: %d\n", (*CAN_TEST) >> 4 & 1);
		}
		default:
			DEBUGOUT("Input Invalid! Try Again.\r\n");
		}
	}
}
