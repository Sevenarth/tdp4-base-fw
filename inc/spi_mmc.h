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
#include "diskio.h"

/* Ports configuration */
#define LPC_SSP       LPC_SSP0
#define MMC_CS_PORT   0
#define MMC_CS_PIN    7
#define MMC_CS_IOCON  IOCON_PIO0_7

#define FAST_BITRATE 2000000 // 2Mbps

/* The SPI data is 8 bit long, the MMC use 48 bits, 6 bytes */
#define MMC_CMD_SIZE 6

/* The max MMC flash size is 256MB */
#define MMC_DATA_SIZE 512 /* 16-bit in size, 512 bytes */

#define MAX_TIMEOUT 0xFF

/* Card type flags (CardType) */
typedef enum card_type {
	CT_NIL = 0x00,            /* No valid card */
	CT_MMC = 0x01,            /* MMC ver 3 */
	CT_SD1 = 0x02,            /* SD ver 1 */
	CT_SD2 = 0x04,            /* SD ver 2 */
	CT_SDC = (CT_SD1|CT_SD2), /* SD */
	CT_BLOCK = 0x08           /* Block addressing */
} card_type_t;

typedef enum mmc_cmd {
	CMD0   = (0x40+0),	/* GO_IDLE_STATE */
	CMD1   = (0x40+1),	/* SEND_OP_COND (MMC) */
	ACMD41 = (0xC0+41),	/* SEND_OP_COND (SDC) */
	CMD8   = (0x40+8),	/* SEND_IF_COND */
	CMD9   = (0x40+9),	/* SEND_CSD */
	CMD10  = (0x40+10),	/* SEND_CID */
	CMD12  = (0x40+12),	/* STOP_TRANSMISSION */
	ACMD13 = (0xC0+13),	/* SD_STATUS (SDC) */
	CMD16  = (0x40+16),	/* SET_BLOCKLEN */
	CMD17  = (0x40+17),	/* READ_SINGLE_BLOCK */
	CMD18  = (0x40+18),	/* READ_MULTIPLE_BLOCK */
	CMD23  = (0x40+23),	/* SET_BLOCK_COUNT (MMC) */
	ACMD23 = (0xC0+23),	/* SET_WR_BLK_ERASE_COUNT (SDC) */
	CMD24  = (0x40+24),	/* WRITE_BLOCK */
	CMD25  = (0x40+25),	/* WRITE_MULTIPLE_BLOCK */
    CMD55  = (0x40+55),	/* APP_CMD */
	CMD58  = (0x40+58)	/* READ_OCR */
} mmc_cmd_t;

card_type_t mmc_init(void);
uint8_t mmc_read_single_block(uint32_t, uint8_t *);
uint8_t mmc_read_multiple_blocks(uint32_t, uint8_t *, unsigned int);
uint8_t mmc_write_single_block(uint32_t, uint8_t *);
uint8_t mmc_write_multiple_blocks(uint32_t, uint8_t *, unsigned int);
int mmc_sync();
int mmc_get_sector_count(uint32_t *);
int mmc_get_block_size(uint32_t *);
int mmc_get_status(uint32_t *);
int mmc_get_ocr(uint32_t *);
int mmc_get_cid(uint32_t *);
int mmc_get_csd(uint32_t *);

#endif /* INC_SPI_MMC_H_ */
