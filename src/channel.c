/**
 * channel.c
 *
 * Contains all the essential functions to
 * communicate to the bank station through
 * CAN bus.
 */
#include "channel.h"
#include "FreeRTOS.h"
#include "task.h"


/**
 * Private definitions
 */

int *sensor_status;
TaskHandle_t *toggleHandle;

volatile uint32_t *CAN_CNTL = (uint32_t *)0x40050000;
volatile uint32_t *CAN_TEST = (uint32_t *)0x40050014;

CCAN_MSG_OBJ_T tx_msg_obj, rx_msg_obj;

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
	*CAN_CNTL |= 0x80; // enable test mode
	*CAN_TEST |= 0x10; // enable loopback mode
}

void chSend32BitValue(uint32_t mode_id, uint32_t val) {
	tx_msg_obj.mode_id = mode_id;
	tx_msg_obj.dlc = 4;
	tx_msg_obj.data[0] = (uint8_t)(val);
	tx_msg_obj.data[1] = (uint8_t)(val >> 8);
	tx_msg_obj.data[2] = (uint8_t)(val >> 16);
	tx_msg_obj.data[3] = (uint8_t)(val >> 24);
	LPC_CCAN_API->can_transmit(&tx_msg_obj);
}

void chSend8BitValue(uint32_t mode_id, uint8_t val) {
	tx_msg_obj.mode_id = mode_id;
	tx_msg_obj.dlc = 1;
	tx_msg_obj.data[0] = val;
	LPC_CCAN_API->can_transmit(&tx_msg_obj);
}

void chSendBuffOverflow() {
	tx_msg_obj.mode_id = 0x100;
	tx_msg_obj.dlc = 0;
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
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	rx_msg_obj.msgobj = msg_obj_num;
	LPC_CCAN_API->can_receive(&rx_msg_obj);
	if (msg_obj_num == 1) {
		switch (rx_msg_obj.mode_id) {
		case 0x0:
			chSend8BitValue(0, (uint8_t) *sensor_status);
			break;
		case 0x3:
			vTaskNotifyGiveFromISR(*toggleHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			break;
		}
	}
}

/*	CAN transmit callback */
void CAN_tx(uint8_t msg_obj_num) {}

/*	CAN error callback */
void CAN_error(uint32_t error_info) {}

void chSetup(int *sensor_status_ptr, TaskHandle_t *toggle_sampling_handle) {
	sensor_status = sensor_status_ptr;
	toggleHandle = toggle_sampling_handle;

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
}
