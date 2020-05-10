#include <stdlib.h>
#include "board.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include "LSM9DS1.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

static TaskHandle_t pushTask, pullTask;

#define STREAM_BUFFER_SZ 64
StreamBufferHandle_t xStreamBuffer;
const size_t xTriggerLevel = 32;

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
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	if(LPC_GPIO[0].MIS & (1 << 7)) { // Interrupt on PIO0_7 triggered
		vTaskNotifyGiveFromISR(pullTask, &xHigherPriorityTaskWoken);
		LPC_GPIO[0].IC |= 1 << 7; // Clear edge-sensitive interrupt
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

static volatile byte_t fifo_status;

#define SENSOR_OUT_SZ 2

static portTASK_FUNCTION(pullSensorData, pvParameters) {
	int i;
	axes_state_t sensor_out[SENSOR_OUT_SZ];

	while(1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		for(i = 0; i < 32; i++) {
			LSM9DS1_Get_G_Output(&sensor_out[0]);
			LSM9DS1_Get_XL_Output(&sensor_out[1]);

			xStreamBufferSend(xStreamBuffer, sensor_out, sizeof(axes_state_t)*SENSOR_OUT_SZ, portMAX_DELAY);
		}

		LSM9DS1_Reset_FIFO();
	}
}

#define WRITE_BUFFER_SZ (4 * 3 /* 4 bytes * 3 axes */ + 1 /* start_byte */)
static volatile size_t buffer_bytes;

static portTASK_FUNCTION(pushSensorData, pvParameters) {
	size_t recv_sz;
	axes_state_t sensor_out[SENSOR_OUT_SZ];
	uint8_t write_buff[WRITE_BUFFER_SZ];
	int i;

	while(1) {
		recv_sz = xStreamBufferReceive(xStreamBuffer, sensor_out, sizeof(axes_state_t)*SENSOR_OUT_SZ, portMAX_DELAY);
		buffer_bytes = xStreamBufferBytesAvailable(xStreamBuffer);

		if(recv_sz > 0) {
			for(i = 0; i < SENSOR_OUT_SZ; i++) {
				write_buff[0] = i;
				write_buff[1] = sensor_out[i].x >> 24;
				write_buff[2] = sensor_out[i].x >> 16;
				write_buff[3] = sensor_out[i].x >> 8;
				write_buff[4] = sensor_out[i].x;
				write_buff[5] = sensor_out[i].y >> 24;
				write_buff[6] = sensor_out[i].y >> 16;
				write_buff[7] = sensor_out[i].y >> 8;
				write_buff[8] = sensor_out[i].y;
				write_buff[9] = sensor_out[i].z >> 24;
				write_buff[10] = sensor_out[i].z >> 16;
				write_buff[11] = sensor_out[i].z >> 8;
				write_buff[12] = sensor_out[i].z;
			}
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
	prvSetupHardware();

	LPC_IOCON->REG[IOCON_PIO0_7] = IOCON_FUNC0 | IOCON_MODE_PULLDOWN; // Set PIO0_3 to GPIO with pull-down resistor
	LPC_GPIO[0].DIR &= ~(1 << 7); // Set PIO0_3 to input
	LPC_GPIO[0].IS  &= ~(1 << 7); // Set edge-sensitive interrupt on PIO0_3
	LPC_GPIO[0].IBE &= ~(1 << 7); // Set single edge interrupt on PIO0_3
	LPC_GPIO[0].IEV |= 1 << 7;          // Set rising edge interrupt on PIO0_3
	LPC_GPIO[0].IE  |= 1 << 7;          // Enable interrupt on PIO0_3
	LPC_GPIO[0].IC |= 1 << 7;

	LSM9DS1_Init();
	LSM9DS1_Set_AG_Interrupt1(INT1_FSS5);
	LSM9DS1_Set_AG_Reg1(G_ODR_952, G_FS_2000, G_BW_0);
	LSM9DS1_Set_AG_Reg6(XL_ODR_952, XL_FS_16, 0, 0);

	LSM9DS1_Set_M_Operating_Mode(SINGLE_CONVERSION_MODE);
	LSM9DS1_Set_M_FS(M_FS_16);
	LSM9DS1_Set_M_Reg1(0, HIGH_PERFORMANCE_MODE, M_ODR_80, 0, 0);
	LSM9DS1_Set_M_Z_Operative_Mode(HIGH_PERFORMANCE_MODE);
	LSM9DS1_Enable_AG_FIFO();

	NVIC_EnableIRQ(EINT0_IRQn); // Enable external interrupts on Port 0

	xTaskCreate(pullSensorData, "pull", 100, NULL, (tskIDLE_PRIORITY + 2UL), &pullTask);
	xTaskCreate(pushSensorData, "push", 175, NULL, (tskIDLE_PRIORITY + 1UL), &pushTask);

    xStreamBuffer = xStreamBufferCreate(STREAM_BUFFER_SZ*sizeof(axes_state_t), xTriggerLevel);

	vTaskStartScheduler();

	Deinit_I2C();

	/* Should never arrive here */
	return 0;
}