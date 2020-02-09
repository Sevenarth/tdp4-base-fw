/*
 * LSM9DS1.h
 *
 * Mappings and helper functions for the
 * STMicroelectronics LSM9DS1 sensor.
 *
 *  Created on: 26 Jan 2020
 *      Author: TDP4 Team 3
 */

#ifndef LSM9DS1_H
#define LSM9DS1_H

#include "chip.h"
#include "i2c.h"

#define LSM9DS1_AG_ADDR 0x6B
#define LSM9DS1_M_ADDR  0x1E

int LSM9DS1_Write_AG_Register(byte_t, byte_t);

byte_t LSM9DS1_Read_Register(byte_t, byte_t);

int LSM9DS1_Init();
int LSM9DS1_Reset();
int LSM9DS1_AG_Reset();
int LSM9DS1_M_Reset();

typedef struct axes_state {
	int x, y, z;
} axes_state_t;

/****************************************
 * Accelerometer (XL) and gyroscope (G) *
 * register mappings         	  	    *
 ****************************************/

#define ACT_THS				0x04 // Activity threshold register
#define ACT_DUR				0x05 // Inactivity duration register

#define INT_GEN_CFG_XL		0x06 /* Linear acceleration sensor interrupt
									generator configuration register */
/*
 * Linear acceleration sensor interrupt threshold registers
 */
#define INT_GEN_THS_X_XL	0x07
#define INT_GEN_THS_Y_XL	0x08
#define INT_GEN_THS_Z_XL	0x09

#define INT_GEN_DUR_XL		0x0A /* Linear acceleration sensor interrupt
									duration register */
#define REFERENCE_G			0x0B /* Angular rate sensor reference value
									register for digital high-pass filter (r/w). */
/*
 * Interrupt control registers
 */

// Register
#define INT1_CTRL			0x0C
// Options
typedef enum int1_ctrl {
	INT1_IG_G    = 0x80, // gyroscope interrupt enable
	INT1_IG_XL   = 0x40, // accelerometer interrupt enable
	INT1_FSS5    = 0x20, // FSS5 (32 FIFO unread samples) interrupt enable
	INT1_OVR     = 0x10, // Overrun (32+ FIFO unread samples) interrupt enable
	INT1_FTH     = 0x08, // FIFO threshold interrupt enable
	INT1_BOOT    = 0x04, // Boot status available enable
	INT1_DRDY_G  = 0x02, // Gyroscope data ready interrupt enable
	INT1_DRDY_XL = 0x01, // Accelerometer data ready interrupt enable
	INT1_UNSET   = 0
} INT1_CTRL_T;

// Sets the interrupt in the sensor INT1_AG. Returns 1 if successful.
int LSM9DS1_Set_AG_Interrupt1(INT1_CTRL_T);

// Register
#define INT2_CTRL			0x0D
// Options
typedef enum int2_ctrl {
	INT2_INACT     = 0x80, // Inactivity interrupt output signal
	INT2_FSS5      = 0x20, // FSS5 (32 FIFO unread samples) interrupt enable
	INT2_OVR	   = 0x10, // Overrun (32+ FIFO unread samples) interrupt enable
	INT2_FTH	   = 0x08, // FIFO threshold interrupt enable
	INT2_DRDY_TEMP = 0x04, // Temperature data ready interrupt enable
	INT2_DRDY_G	   = 0x02, // Gyroscope data ready interrupt enable
	INT2_DRDY_XL   = 0x01, // Accelerometer data ready interrupt enable
	INT2_UNSET     = 0
} INT2_CTRL_T;

// Sets the interrupt in the sensor INT2_AG. Returns 1 if successful.
int LSM9DS1_Set_AG_Interrupt2(INT2_CTRL_T);

#define WHO_AM_I_XG			0x0F // Who am I register, always contains: 0x68

/*
 * Angular rate sensor control registers (G+XL)
 * NOTE: These registers override the XL's
 */

#define CTRL_REG1_G			0x10
// Options
/**
 * Output Data Rate (ODR) for G+XL available as follows:
 * –––––––––––––––––––––––––––––––––––––––
 * | ODR bits | ODR (Hz)   | Cutoff (Hz) |
 * –––––––––––––––––––––––––––––––––––––––
 * | 000      | Power-down | N/A   		 |
 * | 001	  |       14.9 | 5           |
 * | 010      |       59.5 | 19          |
 * | 011      |      119.0 | 38          |
 * | 100      |      238.0 | 76          |
 * | 101      |      476.0 | 100         |
 * | 110      |      952.0 | 100         |
 * –––––––––––––––––––––––––––––––––––––––
 */
typedef enum g_odr {
	G_ODR_POWERDOWN,
	G_ODR_15,
	G_ODR_60,
	G_ODR_119,
	G_ODR_238,
	G_ODR_476,
	G_ODR_952
} G_ODR_T;

/**
 * Gyroscope full-scale selection (dps)
 */
typedef enum g_fs {
	G_FS_245  = 0,
	G_FS_500  = 2,
	G_FS_2000 = 3
} G_FS_T;

/**
 * Gyroscope bandwidth selection (Hz) after LPF2.
 * ––––––––––––––––––––––––––––––––
 * | ODR (Hz) | BW  | Cutoff (Hz) |
 * ––––––––––––––––––––––––––––––––
 * |     59.5 | any | 16          |
 * |    119.0 |  0  | 14          |
 * |    119.0 |  1+ | 31          |
 * |    238.0 |  0  | 14          |
 * |    238.0 |  1  | 29          |
 * |    238.0 |  2  | 63          |
 * |    238.0 |  3  | 78          |
 * |    476.0 |  0  | 21          |
 * |    476.0 |  1  | 28          |
 * |    476.0 |  2  | 57          |
 * |    476.0 |  3  | 100         |
 * |    952.0 |  0  | 33          |
 * |    952.0 |  1  | 40          |
 * |    952.0 |  2  | 58          |
 * |    952.0 |  3  | 100         |
 * ––––––––––––––––––––––––––––––––
 */
typedef enum g_bw {
	G_BW_0,
	G_BW_1,
	G_BW_2,
	G_BW_3
} G_BW_T;

/**
 * Sets CTRL_REG1_G which applies the ODR rate
 * to G+XL, the full-scale for the gyroscope
 * and the bandwidth.
 */
int LSM9DS1_Set_AG_Reg1(G_ODR_T, G_FS_T, G_BW_T);
int LSM9DS1_Set_G_ODR(G_ODR_T);
int LSM9DS1_Set_G_FS(G_FS_T);
int LSM9DS1_Set_G_BW(G_BW_T);

/**
 * LPF+HPF+LPF2, LPF+HPF or LPF multiplexers selector
 */
#define CTRL_REG2_G			0x11
int LSM9DS1_Set_AG_Reg2(uint8_t);

/**
 * High-pass filter control register
 */
#define CTRL_REG3_G			0x12

/**
 * High-pass cutoff frequency selection:
 * ––––––––––––––––––––––––––––––––––––––––––––––––––
 * |     |                  ODR (Hz)                |
 * | Val |–––––––––––––––––––––––––––––––––––––––––––
 * |     | 14.9  | 59.5  | 119  | 238  | 476  | 952 |
 * ––––––––––––––––––––––––––––––––––––––––––––––––––
 * |  0  | 1     | 4     | 8    | 15   | 30   | 57  |
 * |  1  | 0.5   | 2     | 4    | 8    | 15   | 30  |
 * |  2  | 0.2   | 1     | 2    | 4    | 8    | 15  |
 * |  3  | 0.1   | 0.5   | 1    | 2    | 4    | 8   |
 * |  4  | 0.05  | 0.2   | 0.5  | 1    | 2    | 4   |
 * |  5  | 0.02  | 0.1   | 0.2  | 0.5  | 1    | 2   |
 * |  6  | 0.01  | 0.05  | 0.1  | 0.2  | 0.5  | 1   |
 * |  7  | 0.005 | 0.02  | 0.05 | 0.1  | 0.2  | 0.5 |
 * |  8  | 0.002 | 0.01  | 0.02 | 0.05 | 0.1  | 0.2 |
 * |  9  | 0.001 | 0.005 | 0.01 | 0.02 | 0.05 | 0.1 |
 * ––––––––––––––––––––––––––––––––––––––––––––––––––
 */
typedef enum g_hpcf {
	G_HPCF_0,
	G_HPCF_1,
	G_HPCF_2,
	G_HPCF_3,
	G_HPCF_4,
	G_HPCF_5,
	G_HPCF_6,
	G_HPCF_7,
	G_HPCF_8,
	G_HPCF_9
} G_HPCF_T;

/**
 * Sets CTRL_REG_3.
 * @param int Low-power mode enable.
 * @param int High-pass filter enable.
 * @param G_HPCF_T High-pass filter cut-off frequency.
 */
int LSM9DS1_Set_AG_Reg3(int, int, G_HPCF_T);

/**
 * Turns on G+XL high-pass filter with the specified frequency.
 */
int LSM9DS1_Set_AG_HPF(G_HPCF_T);

/**
 * Turns off the G+XL high-pass filter.
 */
int LSM9DS1_Disable_AG_HPF();

#define ORIENT_CFG_G		0x13 /* Angular rate sensor sign and
									orientation register. */

/**
 * Set angular rate sensor sign and orientation register.
 * @param int Pitch axis (X) angular rate sign (0: positive sign; 1: negative sign)
 * @param int Roll axis (Y) angular rate sign (0: positive sign; 1: negative sign)
 * @param int Yaw axis (Z) angular rate sign (0: positive sign; 1: negative sign)
 * @param uint8_t (3-bit) Directional user orientation selection
 */
int LSM9DS1_Set_G_Orientation(int, int, int, uint8_t);

#define INT_GEN_SRC_G		0x14 // Angular rate sensor interrupt source register.

/*
 * Temperature output word in 2's complement.
 */
#define OUT_TEMP_L			0x15
#define OUT_TEMP_H			0x16

int16_t LSM9DS1_Get_AG_Temperature();

#define STATUS_REG_0		0x17 // Status register

/*
 * Gyroscope output words in 2's complement.
 */
#define OUT_X_L_G			0x18
#define OUT_X_H_G			0x19

#define OUT_Y_L_G			0x1A
#define OUT_Y_H_G			0x1B

#define OUT_Z_L_G			0x1C
#define OUT_Z_H_G			0x1D

typedef struct g_state {
	int pitch, roll, yaw;
} g_state_t;

int LSM9DS1_Get_G_Output(g_state_t *);

// G+XL control register
#define CTRL_REG4			0x1E
/**
 * Sets Control Register 4.
 * @param int Gyroscope’s pitch axis (X) output enable
 * @param int Gyroscope’s roll axis (Y) output enable
 * @param int Gyroscope’s yaw axis (Z) output enable
 * @param int Latch XL's interrupt
 * @param int (0: interrupt generator uses 6D for position recognition;
 * 			   1: interrupt generator uses 4D for position recognition)
 */
int LSM9DS1_Set_AG_Reg4(int, int, int, int, int);

/*
 * Linear acceleration sensor control registers (XL)
 * NOTE: overridden by the gyroscope's.
 */
#define CTRL_REG5_XL		0x1F

typedef enum decimation {
	NO_DECIMATION,
	DECIMATION_2_SAMPLES,
	DECIMATION_4_SAMPLES,
	DECIMATION_8_SAMPLES
} DECIMATION_T;

/**
 * Sets Control Register 5 (XL).
 * @param DECIMATION_T Decimation of acceleration data on OUT REG and FIFO.
 * @param int Accelerometer's X-axis output enable
 * @param int Accelerometer's Y-axis output enable
 * @param int Accelerometer's Z-axis output enable
 */
int LSM9DS1_Set_AG_Reg5(DECIMATION_T, int, int, int);

#define CTRL_REG6_XL		0x20

/**
 * Output Data Rate (ODR) for XL-only mode.
 */
typedef enum xl_odr {
	XL_ODR_POWERDOWN,
	XL_ODR_10,
	XL_ODR_50,
	XL_ODR_119,
	XL_ODR_238,
	XL_ODR_476,
	XL_ODR_952
} XL_ODR_T;


/**
 * Accelerometer full-scale selection (g)
 */
typedef enum xl_fs {
	XL_FS_2,  // ±2g
	XL_FS_16, // ±16g
	XL_FS_4,  // ±4g
	XL_FS_8   // ±8g
} XL_FS_T;

/**
 * Accelerometer anti-aliasing filter bandwidth selection (Hz)
 */
typedef enum xl_bw {
	XL_BW_408, // 408Hz
	XL_BW_211, // 211Hz
	XL_BW_105, // 105Hz
	XL_BW_50   // 50Hz
} XL_BW_T;

/**
 * Sets AG Control Register 6 (XL)
 * @param XL_ODR_T Output data rate and power mode selection
 * @param XL_FS_T Accelerometer full-scale selection
 * @param int Bandwidth selection. Default value: 0
				(0: bandwidth determined by ODR selection:
				    - BW = 408 Hz when ODR = 952 Hz, 50 Hz, 10 Hz;
					- BW = 211 Hz when ODR = 476 Hz;
					- BW = 105 Hz when ODR = 238 Hz;
					- BW = 50 Hz when ODR = 119 Hz;
				 1: bandwidth selected according to XL_BW selection)
 */
int LSM9DS1_Set_AG_Reg6(XL_ODR_T, XL_FS_T, int, XL_BW_T);

int LSM9DS1_Set_XL_ODR(XL_ODR_T);
int LSM9DS1_Set_XL_FS(XL_FS_T);
int LSM9DS1_Set_XL_BW_Selector(int);
int LSM9DS1_Set_XL_BW(XL_BW_T);

#define CTRL_REG7_XL		0x21

/*
 * Generic control registers
 */
#define CTRL_REG8			0x22
#define CTRL_REG9			0x23

int LSM9DS1_Enable_AG_FIFO();
/**
 * Enable FIFO threshold level use. Default value: 0
 *  (0: FIFO depth is not limited; 1: FIFO depth is limited to threshold level)
 */
int LSM9DS1_Set_FIFO_Stop_On_Threshold(int);

int LSM9DS1_Reset_FIFO();

#define CTRL_REG10			0x24

#define INT_GEN_SRC_XL		0x26 /* Linear acceleration sensor
									interrupt source register. */

#define STATUS_REG_1		0x27 /* Status register */

/*
 * Accelerometer output words in 2's complement.
 */
#define OUT_X_L_XL			0x28
#define OUT_X_H_XL			0x29

#define OUT_Y_L_XL			0x2A
#define OUT_Y_H_XL			0x2B

#define OUT_Z_L_XL			0x2C
#define OUT_Z_H_XL			0x2D

int LSM9DS1_Get_XL_Output(axes_state_t *);

#define FIFO_CTRL			0x2E /* FIFO control register */

typedef enum fifo_mode {
	BYPASS_MODE = 0,
	FIFO_MODE = 1,
	CONTINUOUS_TO_FIFO = 3,
	BYPASS_TO_CONTINUOUS = 4,
	CONTINUOUS_MODE = 5
} FIFO_MODE_T;

/**
 * Sets FIFO_CTRL register.
 * @param FIFO_MODE_T FIFO mode
 * @param uint8_t (5 bit) FIFO threshold level
 */
int LSM9DS1_Set_AG_FIFO(FIFO_MODE_T, uint8_t);

#define FIFO_SRC			0x2F /* FIFO status register */

/* Angular rate sensor interrupt generator configuration register. */
#define INT_GEN_CFG_G		0x30

/* Angular rate sensor interrupt generator threshold registers.
 * The value is expressed as a 15-bit word in two’s complement.
 */
#define INT_GEN_THS_XH_G	0x31
#define INT_GEN_THS_XL_G	0x32

#define INT_GEN_THS_YH_G	0x33
#define INT_GEN_THS_YL_G	0x34

#define INT_GEN_THS_ZH_G	0x35
#define INT_GEN_THS_ZL_G	0x36

/* Angular rate sensor interrupt generator duration register. */
#define INT_GEN_DUR_G		0x37

/**********************************
 * Magnetometer register mappings *
 **********************************/

/**
 * These registers are 16-bit registers and represent the X/Y/Z offsets used
 * to compensate environmental effects (data is expressed as two’s complement).
 * This value acts on the magnetic output data value in order to subtract the
 * environmental offset.
 */

#define OFFSET_X_REG_L_M	0x05
#define OFFSET_X_REG_H_M	0x06

int LSM9DS1_Set_M_X_Offset(int16_t);

#define OFFSET_Y_REG_L_M	0x07
#define OFFSET_Y_REG_H_M	0x08

int LSM9DS1_Set_M_Y_Offset(int16_t);

#define OFFSET_Z_REG_L_M	0x09
#define OFFSET_Z_REG_H_M	0x0A

int LSM9DS1_Set_M_Z_Offset(int16_t);

#define WHO_AM_I_M			0x0F // Who am I register, always contains: 0x38

#define CTRL_REG1_M			0x20

typedef enum operative_mode {
	LOW_POWER_MODE,
	MEDIUM_PERFORMANCE_MODE,
	HIGH_PERFORMANCE_MODE,
	ULTRA_HIGH_PERFORMANCE_MODE
} OPERATIVE_MODE_T;

typedef enum m_odr {
	M_ODR_0_625, // 0.625Hz
	M_ODR_1_25,  // 1.25Hz
	M_ODR_2_5,   // 2.5Hz
	M_ODR_5,     // 5Hz
	M_ODR_10,    // 10Hz
	M_ODR_20,    // 20Hz
	M_ODR_40,    // 40Hz
	M_ODR_80     // 80Hz
} M_ODR_T;

/**
 * Set Control Register 1.
 * @param int Temperature compensation enable
 * @param OPERATIVE_MODE_T X and Y axes operative mode selection
 * @param M_ODR_T Output data rate selection
 * @param int FAST_ODR enables data rates higher than 80 Hz
 * @param int Self-test enable
 */
int LSM9DS1_Set_M_Reg1(int, OPERATIVE_MODE_T, M_ODR_T, int, int);
int LSM9DS1_Set_M_XY_Operative_Mode(OPERATIVE_MODE_T);
int LSM9DS1_Set_M_ODR(M_ODR_T odr);

#define CTRL_REG2_M			0x21

// Magnetometer full-scale (gauss)
typedef enum m_fs {
	M_FS_4,  // ±4g
	M_FS_8,  // ±8g
	M_FS_12, // ±12g
	M_FS_16  // ±16g
} M_FS_T;

/**
 * Set Control Register 2.
 * @param M_FS_T Full-scale configuration
 * @param int Reboot memory content (0: normal mode; 1: reboot memory content)
 * @param int Configuration registers and user register reset function. (0: default value; 1: reset operation)
 */

int LSM9DS1_Set_M_Reg2(M_FS_T, int, int);
int LSM9DS1_Set_M_FS(M_FS_T);

#define CTRL_REG3_M			0x22

typedef enum m_operating_mode {
	CONTINUOUS_CONVERSION_MODE,
	SINGLE_CONVERSION_MODE,
	POWER_DOWN_MODE
} M_OPERATING_MODE_T;

/**
 * Set Control Register 3.
 * @param int Disable I2C interface
 * @param int Low-power mode configuration
 * @param int SPI Serial Interface mode selection
 * @param M_OPERATING_MODE_T Operating mode selection
 */
int LSM9DS1_Set_M_Reg3(int, int, int, M_OPERATING_MODE_T);
int LSM9DS1_Set_M_Operating_Mode(M_OPERATING_MODE_T);

#define CTRL_REG4_M			0x23

/**
 * Set Control Register 4
 * @param OPERATIVE_MODE Z-axis operative mode selection
 * @param int Big/Little Endian selection (default: 0)
 */
int LSM9DS1_Set_M_Reg4(OPERATIVE_MODE_T, int);
int LSM9DS1_Set_M_Z_Operative_Mode(OPERATIVE_MODE_T);

#define CTRL_REG5_M			0x24
#define STATUS_REG_M		0x27

/*
 * Magnetometer output words in 2's complement.
 */

#define OUT_X_L_M			0x28
#define OUT_X_H_M			0x29

#define OUT_Y_L_M			0x2A
#define OUT_Y_H_M			0x2B

#define OUT_Z_L_M			0x2C
#define OUT_Z_H_M			0x2D

int LSM9DS1_Get_M_Output(axes_state_t *);

#define INT_CFG_M			0x30
#define INT_SRC_M			0x31
#define INT_THS_L_M			0x32
#define INT_THS_H_M			0x33

#endif
