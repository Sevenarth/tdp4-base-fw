#include <stdlib.h>
#include "board.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include "LSM9DS1.h"
#include "channel.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

static TaskHandle_t pushTask, pullTask, toggleTask;

#define STREAM_BUFFER_SZ 64
StreamBufferHandle_t xStreamBuffer;
const size_t xTriggerLevel = 32;

int ss;
bool sampling_active;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

static portTASK_FUNCTION(toggleSampling, pvParameters) {
	while(1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(sampling_active) {
			ss = LSM9DS1_Set_AG_Interrupt1(INT1_UNSET);
			ss = LSM9DS1_Reset_FIFO();
			NVIC_DisableIRQ(EINT0_IRQn);
		} else {
			ss = LSM9DS1_Reset_FIFO();
			ss = LSM9DS1_Set_AG_Interrupt1(INT1_FSS5);
			NVIC_EnableIRQ(EINT0_IRQn);
		}

		sampling_active ^= 1;
	}
}

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
	axes_state_t xl_data, g_data, m_data;
	xl_data.dev = XL;
	g_data.dev = G;
	m_data.dev = M;

	while(1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		for(i = 0; i < 32; i++) {
			LSM9DS1_Get_G_Output(&g_data);
			LSM9DS1_Get_XL_Output(&xl_data);

			xStreamBufferSend(xStreamBuffer, &xl_data, sizeof(axes_state_t), portMAX_DELAY);
			xStreamBufferSend(xStreamBuffer, &g_data, sizeof(axes_state_t), portMAX_DELAY);
		}
		LSM9DS1_Reset_FIFO();

		LSM9DS1_Get_M_Output(&m_data);
		xStreamBufferSend(xStreamBuffer, &m_data, sizeof(axes_state_t), portMAX_DELAY);
	}
}

#define WRITE_BUFFER_SZ (4 * 3 /* 4 bytes * 3 axes */ + 1 /* start_byte */)
static volatile size_t buffer_bytes;

static portTASK_FUNCTION(pushSensorData, pvParameters) {
	size_t recv_sz;
	axes_state_t data;

	while(1) {
		recv_sz = xStreamBufferReceive(xStreamBuffer, &data, sizeof(axes_state_t), portMAX_DELAY);
		buffer_bytes = xStreamBufferBytesAvailable(xStreamBuffer);

		if(recv_sz) {
			chSend32BitValue(CAN_ADDR_OFFSET+data.dev, (uint32_t)data.x);
			chSend32BitValue(CAN_ADDR_OFFSET+data.dev, (uint32_t)data.y);
			chSend32BitValue(CAN_ADDR_OFFSET+data.dev, (uint32_t)data.z);

			if(sizeof(axes_state_t)*STREAM_BUFFER_SZ == buffer_bytes) {
				chSendBuffOverflow();
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
	LPC_GPIO[0].IS  &= ~(1 << 7); // Set edge-sensitive interrupt on PIO0_7
	LPC_GPIO[0].IBE &= ~(1 << 7); // Set single edge interrupt on PIO0_7
	LPC_GPIO[0].IEV |= 1 << 7;    // Set rising edge interrupt on PIO0_7
	LPC_GPIO[0].IE  |= 1 << 7;    // Enable interrupt on PIO0_7
	LPC_GPIO[0].IC |= 1 << 7;

	ss = LSM9DS1_Init();
	ss = LSM9DS1_Set_AG_Reg6(XL_ODR_952, XL_FS_16, 0, 0);
	ss = LSM9DS1_Set_AG_Reg1(G_ODR_952, G_FS_2000, G_BW_0);

	ss = LSM9DS1_Set_M_Operating_Mode(CONTINUOUS_CONVERSION_MODE);
	ss = LSM9DS1_Set_M_FS(M_FS_16);
	ss = LSM9DS1_Set_M_Reg1(0, HIGH_PERFORMANCE_MODE, M_ODR_80, 1, 0);
	ss = LSM9DS1_Set_M_Z_Operative_Mode(HIGH_PERFORMANCE_MODE);
	ss = LSM9DS1_Enable_AG_FIFO();

	chSetup(&ss, &toggleTask);

	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);

	//NVIC_EnableIRQ(EINT0_IRQn); // Enable external interrupts on Port 0

	xTaskCreate(pullSensorData, "pull", 120, NULL, (tskIDLE_PRIORITY + 2UL), &pullTask);
	xTaskCreate(pushSensorData, "push", 100, NULL, (tskIDLE_PRIORITY + 1UL), &pushTask);
	xTaskCreate(toggleSampling, "toggle", 50, NULL, (tskIDLE_PRIORITY + 3UL), &toggleTask);

    xStreamBuffer = xStreamBufferCreate(STREAM_BUFFER_SZ*sizeof(axes_state_t), xTriggerLevel);

	vTaskStartScheduler();

	Deinit_I2C();

	/* Should never arrive here */
	return 0;
}
