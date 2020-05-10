#include "board.h"
#include <stdlib.h>
#include "LSM9DS1.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#ifdef DEBUG_ENABLE
static const char menu[] =
	"**************** LSM9DS1_AG Demo Menu ****************\r\n"
	"\t0: Exit Demo\r\n"
	"\t1: Read output\r\n"
	"\t2: Read register\r\n"
	"\t3: Write register\r\n"
	"\t4: Reset FIFO\r\n";
#endif

/* Get an integer input from UART */
static int con_get_input(const char *str)
{
#ifdef DEBUG_ENABLE
	int input_valid = 0;
	int x;
	char ch[16], *ptr;
	int i = 0;

	while (!input_valid) {
		DEBUGOUT("%s", str);
		while (1) {
			x = DEBUGIN();
			if (x == EOF) {
				continue;
			}
			if (i >= sizeof(ch) - 2) {
				break;
			}
			if (((x == '\r') || (x == '\n')) && i) {
				DEBUGOUT("\r\n");
				break;
			}
			if (x == '\b') {
				if (i) {
					DEBUGOUT("\033[1D \033[1D");
					i--;
				}
				continue;
			}
			DEBUGOUT("%c", x);
			ch[i++] = x;
		}
		ch[i] = 0;
		i = strtol(ch, &ptr, 16);
		if (*ptr) {
			i = 0;
			DEBUGOUT("Invalid input. Retry!\r\n");
			continue;
		}
		input_valid = 1;
	}
	return i;
#else
	static int sind = -1;
	static uint8_t val[] = {5, I2C_SLAVE_IOX_ADDR, 1, 0};
	if (sind >= sizeof(val)) {
		sind = -1;
	}
	while (sind < 0 && (tick_cnt & 0x7F)) {}
	if (sind < 0) {
		sind = 0;
		val[3] = !val[3];
		tick_cnt++;
	}
	return val[sind++];
#endif
}

static int i2c_menu(void)
{
	DEBUGOUT(menu);
	return con_get_input("\r\nSelect an option [0 - 6] :");
}


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
	Board_Init();
	Init_I2C();
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
	int tmp;
	int xflag = 0;

	prvSetupHardware();

	LSM9DS1_Init();
	LSM9DS1_Enable_AG_FIFO();
	LSM9DS1_Set_AG_Interrupt1(INT1_FSS5);
	LSM9DS1_Set_AG_Reg1(G_ODR_952, G_FS_2000, G_BW_0);
	LSM9DS1_Set_AG_Reg6(XL_ODR_952, XL_FS_16, 0, 0);
	LSM9DS1_Set_M_Operating_Mode(SINGLE_CONVERSION_MODE);
	LSM9DS1_Set_M_FS(M_FS_16);
	LSM9DS1_Set_M_Reg1(0, HIGH_PERFORMANCE_MODE, M_ODR_80, 0, 0);
	LSM9DS1_Set_M_Z_Operative_Mode(HIGH_PERFORMANCE_MODE);

	while (!xflag) {
		switch (i2c_menu()) {
		case 0:
			xflag = 1;
			DEBUGOUT("End of LSM9DS1 Demo! Bye!\r\n");
			break;

		case 1:
		{
			g_state_t *g_out = LSM9DS1_Get_G_Output();
			axes_state_t *xl_out = LSM9DS1_Get_XL_Output();
			axes_state_t *m_out = LSM9DS1_Get_M_Output();
			DEBUGOUT("OUTPUTS\r\n\r\n");
			DEBUGOUT("Gyroscope (mdps)\r\n");
			DEBUGOUT("Pitch: %d\tRoll: %d\tYaw: %d\r\n\r\n", g_out->pitch, g_out->roll, g_out->yaw);
			DEBUGOUT("Accelerometer (mg)\r\n");
			DEBUGOUT("X: %d\tY: %d\tZ: %d\r\n\r\n", xl_out->x, xl_out->y, xl_out->z);
			DEBUGOUT("Magnetometer (mgauss)\r\n");
			DEBUGOUT("X: %d\tY: %d\tZ: %d\r\n\r\n", m_out->x, m_out->y, m_out->z);

			free(g_out);
			free(xl_out);
			free(m_out);

			byte_t fifo_status = LSM9DS1_Read_Register(LSM9DS1_AG_ADDR, FIFO_SRC);
			DEBUGOUT("FIFO STATUS\r\n");

			if(fifo_status & 0x80)
				DEBUGOUT("FIFO is at or above threshold\r\n");
			else
				DEBUGOUT("FIFO is below threshold\r\n");

			if(fifo_status & 0x40)
				DEBUGOUT("FIFO is overrunning\r\n");
			else
				DEBUGOUT("FIFO is NOT overrunning\r\n");

			DEBUGOUT("Unread FIFO samples: %u\r\n\r\n", fifo_status & 0x1F);

			break;
		}
		case 2:
			tmp = con_get_input("Register address: ");
			byte_t value = LSM9DS1_Read_Register(LSM9DS1_AG_ADDR, tmp);
			DEBUGOUT("Register value: %x\r\n\r\n", value);
			break;

		case 3:
			tmp = con_get_input("Register address: ");
			int val = con_get_input("Register value: ");
			if(LSM9DS1_Write_AG_Register(tmp, val)) {
				DEBUGOUT("Success!\r\n");
			} else {
				DEBUGOUT("Failed!\r\n");
			}
			break;
		case 4:
			DEBUGOUT("Resetting FIFO...\r\n");
			if(LSM9DS1_Reset_FIFO()) {
				DEBUGOUT("Success!\r\n");
			} else {
				DEBUGOUT("Failed!\r\n");
			}
		default:
			DEBUGOUT("Input Invalid! Try Again.\r\n");
		}
	}
	Deinit_I2C();

	/* Should never arrive here */
	return 0;
}
