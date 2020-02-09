#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include "LSM9DS1.h"
#include "ff.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define RING_BUFFER_SZ 64

static axes_state_t xl_buff[RING_BUFFER_SZ];
static g_state_t g_buff[RING_BUFFER_SZ];
static int data_available = 0, buff_head = 0, buff_tail = 0, buff_full = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/


/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Chip_GPIO_Init(LPC_GPIO);
	Init_I2C();
}

void PIOINT0_IRQHandler() {
	if(LPC_GPIO[0].MIS & 8) { // Interrupt on PIO0_3 triggered
		data_available = 1;
		LPC_GPIO[0].IC |= 8; // Clear edge-sensitive interrupt
	}
}

static portTASK_FUNCTION(pullSensorData, pvParameters) {
	int i;
	while(1) {
		if(data_available) {
			for(i = 0; i < 32; i++) {
				while(buff_full) {}

				LSM9DS1_Get_G_Output(&g_buff[buff_head]);
				LSM9DS1_Get_XL_Output(&xl_buff[buff_head]);

				buff_head = (buff_head + 1) % RING_BUFFER_SZ;
				buff_full = (buff_head == buff_tail);
			}

			data_available = 0;
		}
	}
}

static const char *gyro_out = "gyroscope\t-- pitch:\t%d\troll:\t%d\tyaw:\t%d\r\n";
static const char *xl_printf = "accelerometer\t-- x    :\t%d\ty   :\t%d\tz  :\t%d\r\n\r\n";

static portTASK_FUNCTION(pushSensorData, pvParameters) {
	while(1) {
		if(buff_full || buff_head != buff_tail) {
			DEBUGOUT(gyro_out, g_buff[buff_tail].pitch, g_buff[buff_tail].roll, g_buff[buff_tail].yaw);
			DEBUGOUT(xl_printf, xl_buff[buff_tail].x, xl_buff[buff_tail].y, xl_buff[buff_tail].z);

			buff_tail = (buff_tail + 1) % RING_BUFFER_SZ;
			buff_full = 0;
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
FRESULT open_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
)
{
    FRESULT fr;

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK)
            f_close(fp);
    }
    return fr;
}

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
	prvSetupHardware();

	LPC_IOCON->REG[IOCON_PIO0_3] = IOCON_FUNC0 | IOCON_MODE_PULLDOWN; // Set PIO0_3 to GPIO with pull-down resistor
	LPC_GPIO[0].DIR &= 0xFFFFFFF7; // Set PIO0_3 to input
	LPC_GPIO[0].IS  &= 0xFFFFFFF7; // Set edge-sensitive interrupt on PIO0_3
	LPC_GPIO[0].IBE &= 0xFFFFFFF7; // Set single edge interrupt on PIO0_3
	LPC_GPIO[0].IEV |= 8;          // Set rising edge interrupt on PIO0_3
	LPC_GPIO[0].IE  |= 8;          // Enable interrupt on PIO0_3
	LPC_GPIO[0].IC |= 8;

	LSM9DS1_Init();
	LSM9DS1_Set_AG_Interrupt1(INT1_FSS5);
	LSM9DS1_Set_AG_Reg1(G_ODR_15, G_FS_2000, G_BW_0);
	LSM9DS1_Set_AG_Reg6(XL_ODR_952, XL_FS_16, 0, 0);

	LSM9DS1_Set_M_Operating_Mode(SINGLE_CONVERSION_MODE);
	LSM9DS1_Set_M_FS(M_FS_16);
	LSM9DS1_Set_M_Reg1(0, HIGH_PERFORMANCE_MODE, M_ODR_80, 0, 0);
	LSM9DS1_Set_M_Z_Operative_Mode(HIGH_PERFORMANCE_MODE);
	LSM9DS1_Enable_AG_FIFO();

	NVIC_EnableIRQ(EINT0_IRQn); // Enable external interrupts on Port 0

	xTaskCreate(pullSensorData, "pull", configMINIMAL_STACK_SIZE*2, NULL, (tskIDLE_PRIORITY + 1UL), NULL);
	xTaskCreate(pushSensorData, "push", 103, NULL, (tskIDLE_PRIORITY + 1UL), NULL);

	vTaskStartScheduler();

    FRESULT fr;
    FATFS fs;
    FIL fil;
    unsigned int written;

    /* Open or create a log file and ready to append */
    fr = f_mount(&fs, "", 0);
    fr = open_append(&fil, "20200208T001240Z.TXT");
    if (fr != FR_OK) return 1;

    /* Append a line */
    fr = f_write(&fil, "written\n", 8, &written);

    /* Close the file */
    f_close(&fil);

	while(1) {}

	Deinit_I2C();

	/* Should never arrive here */
	return 0;
}
