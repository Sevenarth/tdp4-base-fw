/*
 * spi_mmc.c
 *
 * Port from https://www.nxp.com/docs/en/application-note/AN10406.pdf
 *
 *  Created on: 1 Feb 2020
 *      Author: TDP4 Team 3
 */

#include "spi_mmc.h"

BYTE mmc_write_data[MMC_DATA_SIZE];
BYTE mmc_read_data[MMC_DATA_SIZE];
BYTE mmc_cmd[MMC_CMD_SIZE];
mmc_status_t mmc_status = NIL;

/************************** MMC Init *********************************/
/*
 * Initialises the MMC into SPI mode and sets block size(512), returns
 * 0 on success
 *
 */
mmc_status_t mmc_init() {
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_8, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* MISO0 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_9, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* MOSI0 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_2, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* SSEL0 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO2_11, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* SCK0 */
	Chip_IOCON_PinLocSel(LPC_IOCON, IOCON_SCKLOC_PIO2_11);

	Chip_SSP_Init(LPC_SSP);

	DWORD i;
	/* Generate a data pattern for write block */
	for(i = 0; i < MMC_DATA_SIZE; i++)
		mmc_write_data[i] = i;

	mmc_status = NIL;

	Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */

	/* initialise the MMC card into SPI mode by sending 80 clks on */
	/* Use MMCRDData as a temporary buffer for SPI_Send() */
	for(i = 0; i < 10; i++)
		mmc_read_data[i] = 0xFF;

	Chip_SSP_WriteFrames_Blocking(LPC_SSP, mmc_read_data, 10);

	Chip_SSP_Disable(LPC_SSP); /* clear SPI SSEL */

	/* send CMD0(RESET or GO_IDLE_STATE) command, all the arguments
  	  are 0x00 for the reset command, precalculated checksum */
	mmc_cmd[0] = 0x40;
	mmc_cmd[1] = 0x00;
	mmc_cmd[2] = 0x00;
	mmc_cmd[3] = 0x00;
	mmc_cmd[4] = 0x00;
	mmc_cmd[5] = 0x95;
	Chip_SSP_WriteFrames_Blocking(LPC_SSP, mmc_cmd, MMC_CMD_SIZE);

	/* if = 1 then there was a timeout waiting for 0x01 from the MMC */
	if(mmc_response(0x01) == 1) {
		mmc_status = IDLE_STATE_TIMEOUT;
		Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */
		return mmc_status;
	}

	/* Send some dummy clocks after GO_IDLE_STATE */
	Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */
	Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, 1);
	Chip_SSP_Disable(LPC_SSP); /* clear SPI SSEL */

	/* must keep sending command until zero response ia back. */
	i = MAX_TIMEOUT;
	do {
		/* send mmc CMD1(SEND_OP_COND) to bring out of idle state */
		/* all the arguments are 0x00 for command one */
		mmc_cmd[0] = 0x41;
		*((uint32_t *) (mmc_cmd+1)) = 0; // set mmc_cmd[1:4] to 0
		/* checksum is no longer required but we always send 0xFF */
		mmc_cmd[5] = 0xFF;

		Chip_SSP_WriteFrames_Blocking(LPC_SSP, mmc_cmd, MMC_CMD_SIZE)
	} while (mmc_response(0x00) != 0 && --i > 0);

	/* timeout waiting for 0x00 from the MMC */
	if ( i == 0 ) {
		mmc_status = OP_COND_TIMEOUT;
		Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */
		return mmc_status;
	}
	/* Send some dummy clocks after SEND_OP_COND */
	Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */

	Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, 1);
	Chip_SSP_Disable(LPC_SSP); /* clear SPI SSEL */

	/* send MMC CMD16(SET_BLOCKLEN) to set the block length */
	mmc_cmd[0] = 0x50;
	mmc_cmd[1] = 0x00; /* 4 bytes from here is the block length */
	/* LSB is first */

	/* 00 00 00 10 set to 16 bytes */
	/* 00 00 02 00 set to 512 bytes */
	mmc_cmd[2] = 0x00;

	/* high block length bits - 512 bytes */
	mmc_cmd[3] = 0x02;

	/* low block length bits */
	mmc_cmd[4] = 0x00;

	/* checksum is no longer required but we always send 0xFF */
	mmc_cmd[5] = 0xFF;

	Chip_SSP_WriteFrames_Blocking(LPC_SSP, mmc_cmd, MMC_CMD_SIZE);

	if(mmc_response(0x00) == 1) {
		mmc_status = SET_BLOCKLEN_TIMEOUT;
		Chip_SSP_Enable(LPC_SSP); /*set SPI SSEL*/
		return mmc_status;
	}

	Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */
	Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, 1);

	return 0;
}

/************************** MMC Write Block ***************************/
/* write a block of data based on the length that has been set
 * in the SET_BLOCKLEN command.
 * Send the WRITE_SINGLE_BLOCK command out first, check the
 * R1 response, then send the data start token(bit 0 to 0) followed by
 * the block of data. The test program sets the block length to 512
 * bytes. When the data write finishs, the response should come back
 * as 0xX5 bit 3 to 0 as 0101B, then another non-zero value indicating
 * that MMC card is in idle state again.
 *
 */
int mmc_write_block(WORD block_number) {
	WORD varl, varh;
	BYTE Status;

	Chip_SSP_Disable(LPC_SSP); /* clear SPI SSEL */

	/* block size has been set in mmc_init() */
	varl = (block_number & 0x003F) << 9;
	varh = (block_number & 0xFFC0) >> 7;

	/* send mmc CMD24(WRITE_SINGLE_BLOCK) to write the data to MMC card */
	mmc_cmd[0] = 0x58;

	/* high block address bits, varh HIGH and LOW */
	mmc_cmd[1] = varh >> 0x08;
	mmc_cmd[2] = varh & 0xFF;

	/* low block address bits, varl HIGH and LOW */
	mmc_cmd[3] = varl >> 0x08;
	mmc_cmd[4] = varl & 0xFF;

	/* checksum is no longer required but we always send 0xFF */
	mmc_cmd[5] = 0xFF;

	Chip_SSP_WriteFrames_Blocking(LPC_SSP, mmc_cmd, MMC_CMD_SIZE);

	/* if mmc_response returns 1 then we failed to get a 0x00 response */
	if(mmc_response(0x00) == 1) {
		mmc_status = WRITE_BLOCK_TIMEOUT;
		Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */
		return mmc_status;
	}

	/* Set bit 0 to 0 which indicates the beginning of the data block */
	mmc_cmd[0] = 0xFE;

	Chip_SSP_WriteFrames_Blocking(LPC_SSP, mmc_cmd, 1);

	/* send data, pattern as 0x00,0x01,0x02,0x03,0x04,0x05 ...*/
	Chip_SSP_WriteFrames_Blocking(LPC_SSP, mmc_write_data, MMC_DATA_SIZE);

	/* Send dummy checksum */
	/* when the last check sum is sent, the response should come back
  	   immediately. So, check the SPI FIFO MISO and make sure the status
  	   return 0xX5, the bit 3 through 0 should be 0x05 */
	mmc_cmd[0] = 0xFF;
	mmc_cmd[1] = 0xFF;
	Chip_SSP_WriteFrames_Blocking(LPC_SSP, mmc_cmd, 2);

	Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, 1);
	status = mmc_read_data[0];

	if((status & 0x0F) != 0x05) {
		mmc_status = WRITE_BLOCK_FAIL;
		Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */
		return mmc_status;
	}

	/* if the status is already zero, the write hasn't finished yet and card is busy */
	if(mmc_wait_for_write_finish() == 1) {
		mmc_status = WRITE_BLOCK_FAIL;
		Chip_SSP_Enable(LPC_SSP);
		return mmc_status;
	}

	Chip_SSP_Enable(LPC_SSP);
	Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, 1);
	return 0;
}

/************************** MMC Read Block ****************************/
/*
 * Reads a 512 Byte block from the MMC
 * Send READ_SINGLE_BLOCK command first, wait for response come back
 * 0x00 followed by 0xFE. The call SPI_Receive() to read the data
 * block back followed by the checksum.
 *
 */
int mmc_read_block(WORD block_number) {
	WORD Checksum;
	WORD varh, varl;

	Chip_SSP_Disable(LPC_SSP); /* clear SPI SSEL */

	varl = (block_number & 0x003F) << 9;
	varh = (block_number & 0xFFC0) >> 7;

	/* send MMC CMD17(READ_SINGLE_BLOCK) to read the data from MMC card */
	mmc_cmd[0] = 0x51;
	/* high block address bits, varh HIGH and LOW */
	mmc_cmd[1] = varh >> 0x08;
	mmc_cmd[2] = varh & 0xFF;
	/* low block address bits, varl HIGH and LOW */
	mmc_cmd[3] = varl >> 0x08;
	mmc_cmd[4] = varl & 0xFF;
	/* checksum is no longer required but we always send 0xFF */
	mmc_cmd[5] = 0xFF;

	Chip_SSP_WriteFrames_Blocking(LPC_SSP, mmc_cmd, MMC_CMD_SIZE);

	/* if mmc_response returns 1 then we failed to get a 0x00 response */
	if(mmc_response(0x00) == 1) {
		mmc_status = READ_BLOCK_TIMEOUT;
		Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */
		return mmc_status;
	}
	/* wait for data token */
	if(mmc_response(0xFE) == 1) {
		mmc_status = READ_BLOCK_DATA_TOKEN_MISSING;
		Chip_SSP_Enable(LPC_SSP);
		return mmc_status;
	}

	/* Get the block of data based on the length */
	Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, MMC_DATA_SIZE);

	/* CRC bytes that are not needed */
	Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, 2);
	Checksum = mmc_read_data[0] << 0x08 | mmc_read_data[1];

	Chip_SSP_Enable(LPC_SSP); /* set SPI SSEL */
	Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, 1) ;

	return 0;
}
/***************** MMC get response *******************/
/*
 * Repeatedly reads the MMC until we get the
 * response we want or timeout
 */
int mmc_response(BYTE response) {
	if(Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, 1) && mmc_read_data[0] == response) {
		return 0;
	}

	return 1;
}

/***************** MMC wait for write finish *******************/
/*
 * Repeatedly reads the MMC until we get a non-zero value (after
 * a zero value) indicating the write has finished and card is no
 * longer busy.
 *
 */
int mmc_wait_for_write_finish() {
	DWORD count = 0xFFFF; /* The delay is set to maximum considering the longest data block length to handle */
	BYTE result = 0;

	while((result == 0) && count)
	{
		Chip_SSP_ReadFrames_Blocking(LPC_SSP, mmc_read_data, 1);
		result = mmc_read_data[0];
		count--;
	}

	if(count == 0)
		return 1;

	return 0;
}
