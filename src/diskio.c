/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "spi_mmc.h"

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

DSTATUS status = STA_NOINIT;
card_type_t card_type;

DSTATUS disk_initialize(BYTE drv) {
	if(drv) return STA_NODISK; // Only 1 drive (drv=0)

	card_type = mmc_init();

	if(card_type) {
		status &= ~STA_NOINIT;
	}

	return status;
}

DSTATUS disk_status (BYTE drv) {
	if (drv) return STA_NOINIT;		/* Supports only single drive */
	return status;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,			/* Physical drive nmuber (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	LBA_t sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..255) */
)
{
	if (drv || !count) return RES_PARERR;
	if (status & STA_NOINIT) return RES_NOTRDY;

	if (!(card_type & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	if (count == 1) {  /* Single block read */
		count = mmc_read_single_block(sector, buff);
	} else {	      /* Multiple block read */
		count = mmc_read_multiple_blocks(sector, buff, count);
	}

	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0) */
	const BYTE *buff,	/* Pointer to the data to be written */
	LBA_t sector,		/* Start sector number (LBA) */
	UINT count			/* Sector count (1..255) */
)
{
	if (drv || !count) return RES_PARERR;
	if (status & STA_NOINIT) return RES_NOTRDY;
	if (status & STA_PROTECT) return RES_WRPRT;

	if (!(card_type & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	if (count == 1) {	/* Single block write */
		count = mmc_write_single_block(sector, buff);
	} else {				/* Multiple block write */
		count = mmc_write_multiple_blocks(sector, buff, count);
	}

	return count ? RES_ERROR : RES_OK;
}

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;

	if (drv) return RES_PARERR;

	res = RES_ERROR;

	if (status & STA_NOINIT) return RES_NOTRDY;

	switch (ctrl) {
	case CTRL_SYNC :		/* Make sure that no pending write process. Do not remove this or written sector might not left updated. */
		if (mmc_sync()) {
			res = RES_OK;
		}
		break;

	case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
		if(mmc_get_sector_count(buff)) {
			res = RES_OK;
		}
		break;

	case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
		*(WORD*)buff = 512;
		res = RES_OK;
		break;

	case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
		if(mmc_get_block_size(buff)) {
			res = RES_OK;
		}
		break;

	case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
		*(BYTE*)buff = card_type;
		res = RES_OK;
		break;

	case MMC_GET_CSD :		/* Receive CSD as a data block (16 bytes) */
		if(mmc_get_csd(buff)) {
			res = RES_OK;
		}
		break;

	case MMC_GET_CID :		/* Receive CID as a data block (16 bytes) */
		if(mmc_get_cid(buff)) {
			res = RES_OK;
		}
		break;

	case MMC_GET_OCR :		/* Receive OCR as an R3 resp (4 bytes) */
		if (mmc_get_ocr(buff)) {	/* READ_OCR */
			res = RES_OK;
		}
		break;

	case MMC_GET_SDSTAT :	/* Receive SD status as a data block (64 bytes) */
		if (mmc_get_status(buff)) {
			res = RES_OK;
		}
		break;

	default:
		res = RES_PARERR;
	}

	return res;
}
