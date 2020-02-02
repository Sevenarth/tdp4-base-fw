/*
 * spi_mmc.h
 *
 *  Created on: 1 Feb 2020
 *      Author: sowia
 */

#ifndef INC_SPI_MMC_H_
#define INC_SPI_MMC_H_

#include "chip.h"
#include "ff.h"

#define LPC_SSP LPC_SSP0
#define MMC_CS

/* The SPI data is 8 bit long, the MMC use 48 bits, 6 bytes */
#define MMC_CMD_SIZE 6

/* The max MMC flash size is 256MB */
#define MMC_DATA_SIZE 512 /* 16-bit in size, 512 bytes */

#define MAX_TIMEOUT 0xFF

typedef enum mmc_status {
	NIL,
	IDLE_STATE_TIMEOUT,
	OP_COND_TIMEOUT,
	SET_BLOCKLEN_TIMEOUT,
	WRITE_BLOCK_TIMEOUT,
	WRITE_BLOCK_FAIL,
	READ_BLOCK_TIMEOUT,
	READ_BLOCK_DATA_TOKEN_MISSING,
	DATA_TOKEN_TIMEOUT,
	SELECT_CARD_TIMEOUT,
	SET_RELATIVE_ADDR_TIMEOUT
} mmc_status_t;

int mmc_init(void);
int mmc_response(BYTE response);
int mmc_read_block(WORD block_number);
int mmc_write_block(WORD block_number);
int mmc_wait_for_write_finish(void);

#endif /* INC_SPI_MMC_H_ */
