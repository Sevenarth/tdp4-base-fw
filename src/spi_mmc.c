/*
 * spi_mmc.c
 *
 * Port from https://www.nxp.com/docs/en/application-note/AN10406.pdf
 * 	         https://github.com/microbuilder/LPC1114CodeBase/blob/master/drivers/fatfs/mmc.c
 *
 *  Created on: 1 Feb 2020
 *      Author: TDP4 Team 3
 */

#define IOCON_GPIO (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_INV_EN | IOCON_DIGMODE_EN)
#include "spi_mmc.h"

static uint8_t tx_buffer[MMC_DATA_SIZE];
static uint8_t rx_buffer[MMC_DATA_SIZE];
static uint8_t card_type;

/**
 * Private definitions
 */

void ssp_init() {
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_8, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* MISO0 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_9, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* MOSI0 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, MMC_CS_IOCON, IOCON_GPIO);	                        /* SSEL0 */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, MMC_CS_PORT, MMC_CS_PIN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO2_11, (IOCON_FUNC1 | IOCON_MODE_INACT));	/* SCK0 */
	Chip_IOCON_PinLocSel(LPC_IOCON, IOCON_SCKLOC_PIO2_11);

	Chip_SSP_Init(LPC_SSP);
	Chip_SSP_Enable(LPC_SSP);
}

int32_t ssp_send(uint32_t buffer_len) {
	return Chip_SSP_WriteFrames_Blocking(LPC_SSP, tx_buffer, buffer_len);
}

int32_t ssp_read(uint32_t buffer_len) {
	return Chip_SSP_ReadFrames_Blocking(LPC_SSP, rx_buffer, buffer_len);
}

#define ssp_recv() ssp_read(1)

static uint8_t wait_ready(void)
{
	uint8_t timeout = MAX_TIMEOUT;

	ssp_recv();

	do {
		ssp_recv();
	} while (*rx_buffer != 0xFF && timeout--);

	return *rx_buffer;
}

static void card_deselect() {
	LPC_GPIO[MMC_CS_PORT].DATA[1 << MMC_CS_PIN] = (1 << MMC_CS_PIN);
	ssp_recv();
}

static uint8_t card_select() {
	LPC_GPIO[MMC_CS_PORT].DATA[1 << MMC_CS_PIN] = 0;

	if(wait_ready() != 0xFF) {
		card_deselect();
		return 0;
	}

	return 1;
}

uint8_t mmc_send_cmd(mmc_cmd_t cmd, uint32_t arg) {
	uint8_t n, res;

	if (cmd & 0x80) { /* ACMD<n> is the command sequence of CMD55-CMD<n> */
		cmd &= 0x7F;
		res = mmc_send_cmd(CMD55, 0);

		if (res > 1) return res;
	}

	card_deselect();
	if (!card_select()) return 0xFF;

	tx_buffer[0] = cmd;
	tx_buffer[1] = (uint8_t) (arg >> 24);
	tx_buffer[2] = (uint8_t) (arg >> 16);
	tx_buffer[3] = (uint8_t) (arg >> 8);
	tx_buffer[4] = (uint8_t) arg;
	switch(cmd) { // CRC
	case CMD0: { tx_buffer[5] = 0x95; break; }  // crc for CMD0
	case CMD8: { tx_buffer[5] = 0x87; break; }  // crc for CMD8
	default: { tx_buffer[5] = 0x01; }           // crc not checked
	}

	ssp_send(6);

	if(cmd == CMD12) ssp_recv(); // skip a byte

	n = 10;
	do ssp_recv(); while ((*rx_buffer & 0x80) && --n);

	return *rx_buffer;
}

static int mmc_read_datablock (
	uint8_t *buff,			/* Data buffer to store received data */
	unsigned int btr		/* Byte count (must be multiple of 4) */
)
{
	uint8_t token, timeout = 20;
	do {
		ssp_recv();
		token = *rx_buffer;
	} while ((token == 0xFF) && timeout--);

	if(token != 0xFE) return 0;	/* If not valid data token, return with error */

	do {							/* Receive the data block into buffer */
		Chip_SSP_ReadFrames_Blocking(LPC_SSP, buff, 4);
		buff += 4;
	} while (btr -= 4);

	ssp_recv();		/* Discard CRC */
	ssp_recv();

	return 1;	/* Return with success */
}

static int mmc_write_datablock (
	uint8_t *buff,	/* 512 byte data block to be transmitted */
	uint8_t token			/* Data/Stop token */
)
{
	if (wait_ready() != 0xFF) return 0;

	*tx_buffer = token;
	ssp_send(1);					/* Xmit data token */

	if (token != 0xFD) {	/* Is data token */
		Chip_SSP_WriteFrames_Blocking(LPC_SSP, buff, 512);

		tx_buffer[0] = 0xFF;
		tx_buffer[1] = 0xFF;
		ssp_send(2);					/* CRC (Dummy) */

		ssp_recv();				/* Reveive data response */
		if ((*rx_buffer & 0x1F) != 0x05)		/* If not accepted, return with error */
			return 0;
	}

	return 1;
}

card_type_t mmc_init() {
	ssp_init();
	card_deselect();

	uint8_t n, cmd, ocr[4], timeout;

	/* initialise the MMC card into SPI mode by sending 80 clks on */
	/* Use MMCRDData as a temporary buffer for SPI_Send() */
	ssp_read(10);

	/* send CMD0(RESET or GO_IDLE_STATE) command, all the arguments
  	  are 0x00 for the reset command */
	if (mmc_send_cmd(CMD0, 0) == 0x01) {
		timeout = 100;
		if (mmc_send_cmd(CMD8, 0x1AA) == 0x01) { /* SDHC */
			for (n = 0; n < 4; n++) {
				ssp_recv(); /* Get trailing return value of R7 resp */
				ocr[n] = *rx_buffer;
			}

			if (ocr[2] == 0x01 && ocr[3] == 0xAA) { /* The card can work at vdd range of 2.7-3.6V */
				while (timeout-- && mmc_send_cmd(ACMD41, 1UL << 30)); /* Wait for leaving idle state (ACMD41 with HCS bit) */

				if (timeout && mmc_send_cmd(CMD58, 0) == 0) { /* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) {
						ssp_recv();
						ocr[n] = *rx_buffer;
					}

					card_type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
				}
			}
		} else { /* SDSC or MMC */
			if (mmc_send_cmd(ACMD41, 0) <= 1) 	{
				card_type = CT_SD1;
				cmd = ACMD41; /* SDv1 */
			} else {
				card_type = CT_MMC;
				cmd = CMD1; /* MMCv3 */
			}

			while (timeout-- && mmc_send_cmd(cmd, 0));       /* Wait for leaving idle state */
			if (!timeout || mmc_send_cmd(CMD16, 512) != 0)   /* Set R/W block length to 512 */
				card_type = 0;
		}
	}

	card_deselect();

	if(card_type) {
		Chip_SSP_SetBitRate(LPC_SSP, FAST_BITRATE);
	}

	return card_type;
}

uint8_t mmc_read_single_block(uint32_t sector, uint8_t *buffer) {
	int success = (mmc_send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
						&& mmc_read_datablock(buffer, 512);
	card_deselect();

	if(success)
		return 0;

	return 1;
}

uint8_t mmc_read_multiple_blocks(uint32_t sector, uint8_t *buffer, unsigned int length) {
	if (mmc_send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
		do {
			if (!mmc_read_datablock(buffer, 512)) break;
			buffer += 512;
		} while (--length);
		mmc_send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
	}

	card_deselect();

	return length;
}

uint8_t mmc_write_single_block(uint32_t sector, uint8_t *buffer) {
	int success = (mmc_send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
						&& mmc_write_datablock(buffer, 0xFE);
	card_deselect();

	if(success)
		return 0;

	return 1;
}

uint8_t mmc_write_multiple_blocks(uint32_t sector, uint8_t *buffer, unsigned int length) {
	if (card_type & CT_SDC) mmc_send_cmd(ACMD23, length);

	if (mmc_send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
		do {
			if (!mmc_write_datablock(buffer, 0xFC)) break;
			buffer += 512;
		} while (--length);

		if (!mmc_write_datablock(0, 0xFD))	/* STOP_TRAN token */
			length = 1;
	}

	card_deselect();

	return length;
}

int mmc_sync() {
	if(card_select()) {
		card_deselect();
		return 1;
	}

	return 0;
}

int mmc_get_sector_count(uint32_t *buff) {
	uint8_t n, csd[16];
	int success = 0;

	if ((mmc_send_cmd(CMD9, 0) == 0) && mmc_read_datablock(csd, 16)) {
		uint16_t csize;

		if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
			csize = (csd[9] + ((uint16_t) csd[8] << 8) + 1);
			*buff = (uint32_t) csize << 10;
		} else {					/* SDC ver 1.XX or MMC*/
			n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
			csize = (csd[8] >> 6) + ((uint16_t) csd[7] << 2) + ((uint16_t) (csd[6] & 3) << 10) + 1;
			*buff = (uint32_t) csize << (n - 9);
		}

		success = 1;
	}

	card_deselect();
	return success;
}

int mmc_get_block_size(uint32_t *buffer) {
	uint8_t n, csd[16];
	int success = 0;

	if (card_type & CT_SD2) {	/* SDC ver 2.00 */
		if (mmc_send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
			ssp_recv();
			if (mmc_read_datablock(csd, 16)) {		/* Read partial block */
				for (n = 64 - 16; n; n--) ssp_recv();	/* Purge trailing data */
				*buffer = 16UL << (csd[10] >> 4);

				success = 1;
			}
		}
	} else {					/* SDC ver 1.XX or MMC */
		if ((mmc_send_cmd(CMD9, 0) == 0) && mmc_read_datablock(csd, 16)) {	/* Read CSD */
			if (card_type & CT_SD1) {	/* SDC ver 1.XX */
				*buffer = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
			} else {					/* MMC */
				*buffer = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
			}

			success = 1;
		}
	}

	card_deselect();
	return success;
}

int mmc_get_status(uint32_t *buffer) {
	int success = 0;

	if (mmc_send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
		ssp_recv();
		if (mmc_read_datablock(buffer, 64))
			success = 1;
	}

	card_deselect();
	return success;
}

int mmc_get_ocr(uint32_t *buffer) {
	int success = 0, n;
	if (mmc_send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
		for (n = 4; n; n--) {
			ssp_recv();
			*buffer++ = *rx_buffer;
		}
		success = 1;
	}

	card_deselect();
	return success;
}

int mmc_get_cid(uint32_t *buffer) {
	int success = 0;

	if (mmc_send_cmd(CMD10, 0) == 0		/* READ_CID */
				&& mmc_read_datablock(buffer, 16))
				success = 1;

	card_deselect();
	return success;
}

int mmc_get_csd(uint32_t *buffer) {
	int success = 0;

	if (mmc_send_cmd(CMD9, 0) == 0		/* READ_CSD */
				&& mmc_read_datablock(buffer, 16))
				success = 1;

	card_deselect();
	return success;
}
