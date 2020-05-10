/*
 * i2c.c
 *
 *  Created on: 28 Jan 2020
 *      Author: TDP4 Team 3
 */

#include "i2c.h"

static uint8_t tx_buffer[128];

void I2C_IRQHandler() {
	if (Chip_I2C_IsMasterActive(I2C0)) {
		Chip_I2C_MasterStateHandler(I2C0);
	} else {
		Chip_I2C_SlaveStateHandler(I2C0);
	}
}

void Init_I2C() {
	// Set up I2C I/O
	Chip_SYSCTL_PeriphReset(RESET_I2C0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_4, IOCON_FUNC1 | IOCON_FASTI2C_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_5, IOCON_FUNC1 | IOCON_FASTI2C_EN);

	// Initialise I2C protocol
	Chip_I2C_Init(I2C0);
	Chip_I2C_SetClockRate(I2C0, DEFAULT_CLK);

	Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandler);
	NVIC_EnableIRQ(I2C0_IRQn);
}

void Deinit_I2C() {
	Chip_I2C_DeInit(I2C0);

	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_4, IOCON_FUNC0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_5, IOCON_FUNC0);
}


int I2C_Send_Byte(byte_t addr, byte_t byte) {
	tx_buffer[0] = byte;
	return Chip_I2C_MasterSend(I2C0, addr, tx_buffer, 1);
}

int I2C_Send_Bytes(byte_t addr, byte_str_t *bytes) {
	return Chip_I2C_MasterSend(I2C0, addr, bytes->bytes, bytes->size);
}

/**
 * Read bytes after sending a command.
 */
int I2C_Cmd_Read_Bytes(byte_t addr, byte_t cmd, byte_str_t *bytes) {
	return Chip_I2C_MasterCmdRead(I2C0, addr, cmd, bytes->bytes, bytes->size);
}
