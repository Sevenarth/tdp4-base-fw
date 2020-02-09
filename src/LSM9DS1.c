/*
 * LSM9DS1.c
 *
 *  Created on: 28 Jan 2020
 *      Author: TDP4 Team 3
 */

#include "LSM9DS1.h"
#include <stdlib.h>
#include <string.h>

/**
 * Private definitions
 */

// Register sizes
#define LSM9DS1_AG_MEM_SIZE 0x38
#define LSM9DS1_M_MEM_SIZE  0x34

// Registers
static uint8_t LSM9DS1_AG[LSM9DS1_AG_MEM_SIZE];
static uint8_t LSM9DS1_M[LSM9DS1_M_MEM_SIZE];

/************************
 * Sensitivity dividers *
 ************************/

#define XL_SENSITIVITY_2  16393
#define XL_SENSITIVITY_4   8196
#define XL_SENSITIVITY_8   4098
#define XL_SENSITIVITY_16  1366

#define G_SENSITIVITY_245   114
#define G_SENSITIVITY_500    57
#define G_SENSITIVITY_2000   14

#define M_SENSITIVITY_4    7143
#define M_SENSITIVITY_8    3448
#define M_SENSITIVITY_12   2326
#define M_SENSITIVITY_16   1724

static int gDiv = 1, xlDiv = 1, mDiv = 1;

void set_g_divider(G_FS_T fs) {
	switch(fs) {
	case G_FS_245:
	{
		gDiv = G_SENSITIVITY_245;
		break;
	}
	case G_FS_500:
	{
		gDiv = G_SENSITIVITY_500;
		break;
	}
	case G_FS_2000:
	{
		gDiv = G_SENSITIVITY_2000;
	}
	}
}

void set_xl_divider(XL_FS_T fs) {
	switch(fs) {
	case XL_FS_2:
	{
		xlDiv = XL_SENSITIVITY_2;
		break;
	}
	case XL_FS_4:
	{
		xlDiv = XL_SENSITIVITY_4;
		break;
	}
	case XL_FS_8:
	{
		xlDiv = XL_SENSITIVITY_8;
		break;
	}
	case XL_FS_16:
	{
		xlDiv = XL_SENSITIVITY_16;
	}
	}
}

void set_m_divider(M_FS_T fs) {
	switch(fs) {
	case M_FS_4:
	{
		mDiv = M_SENSITIVITY_4;
		break;
	}
	case M_FS_8:
	{
		mDiv = M_SENSITIVITY_8;
		break;
	}
	case M_FS_12:
	{
		mDiv = M_SENSITIVITY_12;
		break;
	}
	case M_FS_16:
	{
		mDiv = M_SENSITIVITY_16;
	}
	}
}

int LSM9DS1_Get_Axes_Raw_Output(byte_t addr, byte_t start_reg, int auto_increment, int16_t *buffer) {
	int k = 0, i;

	if(auto_increment) { // auto increment turned on
		byte_t bytes[] = {0, 0, 0, 0, 0, 0};
		byte_str_t byteStr = {6, bytes};
		if(I2C_Cmd_Read_Bytes(addr, start_reg, &byteStr) == 6) {
			buffer[0] = byteStr.bytes[0] | (byteStr.bytes[1] << 8);
			buffer[1] = byteStr.bytes[2] | (byteStr.bytes[3] << 8);
			buffer[2] = byteStr.bytes[4] | (byteStr.bytes[5] << 8);
			k = 3;
		}
	} else {
		byte_t bytes[] = {0, 0};
		byte_str_t byteStr = {2, bytes};
		for(i = 0; i < 3; i++) {
			if(I2C_Cmd_Read_Bytes(addr, start_reg+(2*i), &byteStr) & 2) {
				buffer[i] = byteStr.bytes[0] | (byteStr.bytes[1] << 8);
				k++;
			}
		}
	}

	return k;
}

int LSM9DS1_Get_Axes_Output(byte_t addr, byte_t start_reg, int auto_increment, int sensitivity_divider, axes_state_t *out) {
	int16_t buff16[] = {0, 0, 0};
	int read_bytes = LSM9DS1_Get_Axes_Raw_Output(addr, start_reg, auto_increment, buff16);

	// We need to remodel the data from 16 bits to 32 bits, in order
	// to provide a higher accuracy using a smaller scale (milli/10e-3)
	out->x = buff16[0]*1000/sensitivity_divider;
	out->y = buff16[1]*1000/sensitivity_divider;
	out->z = buff16[2]*1000/sensitivity_divider;

	return read_bytes;
}

int LSM9DS1_Write_Register(byte_t addr, byte_t reg, byte_t value) {
	byte_t bytes[] = {reg, value};
	byte_str_t req = {2, bytes};
	return I2C_Send_Bytes(addr, &req) & 2;
}

int LSM9DS1_Write_M_Register(byte_t reg, byte_t value) {
	int ret = LSM9DS1_Write_Register(LSM9DS1_M_ADDR, reg, value);
	if(ret) {
		LSM9DS1_M[reg] = value;
	}
	return ret;
}

int LSM9DS1_Write_AG_Register(byte_t reg, byte_t value) {
	int ret = LSM9DS1_Write_Register(LSM9DS1_AG_ADDR, reg, value);
	if(ret) {
		LSM9DS1_AG[reg] = value;
	}
	return ret;
}

/**
 * Public definitions
 */

int LSM9DS1_Init() {
	return LSM9DS1_Reset();
}

int LSM9DS1_Reset() {
	int ret = LSM9DS1_AG_Reset();
	ret |= LSM9DS1_M_Reset();
	return ret;
}

int LSM9DS1_AG_Reset() {
	// Setting default values for gyroscope+accelerometer registers
	memset(LSM9DS1_AG, 0, sizeof(uint8_t)*LSM9DS1_AG_MEM_SIZE);
	LSM9DS1_AG[0x0F] = 0x68;
	LSM9DS1_AG[0x1E] = 0x38;
	LSM9DS1_AG[0x1F] = 0x38;
	LSM9DS1_AG[0x22] = 0x04;

	return LSM9DS1_Write_Register(LSM9DS1_AG_ADDR, CTRL_REG8, 1); // AG soft reset trigger
}

int LSM9DS1_M_Reset() {
	// Setting default values for magnetometer registers
	memset(LSM9DS1_M, 0, sizeof(uint8_t)*LSM9DS1_M_MEM_SIZE);
	LSM9DS1_M[0x0F] = 0x38;
	LSM9DS1_M[0x20] = 0x10;
	LSM9DS1_M[0x22] = 0x03;
	LSM9DS1_M[0x30] = 0x08;

	return LSM9DS1_Write_Register(LSM9DS1_M_ADDR, CTRL_REG2_M, 4); // M soft reset trigger
}

byte_t LSM9DS1_Read_Register(byte_t addr, byte_t reg) {
	byte_t bytes[] = {0};
	byte_str_t byteStr = {1, bytes};
	if(I2C_Cmd_Read_Bytes(addr, reg, &byteStr)) {
		return byteStr.bytes[0];
	}

	return 0;
}

int LSM9DS1_Reset_FIFO() {
	if(LSM9DS1_AG[FIFO_CTRL] & 0x20) {
		int ret;

		byte_t bytes[] = {FIFO_CTRL, 0};
		byte_str_t req = {2, bytes};

		ret = I2C_Send_Bytes(LSM9DS1_AG_ADDR, &req);

		req.bytes[1] = LSM9DS1_AG[FIFO_CTRL]; // restore previous register data
		ret &= I2C_Send_Bytes(LSM9DS1_AG_ADDR, &req);

		return ret & 2; // returns 2 if successful, otherwise 0
	}

	return 0;
}

int LSM9DS1_Enable_AG_FIFO() {
	if(!(LSM9DS1_AG[FIFO_CTRL] & 0x20)) {
		int tmp = LSM9DS1_Write_AG_Register(FIFO_CTRL, (LSM9DS1_AG[FIFO_CTRL] & 0x1F) | 0x20);
		tmp &= LSM9DS1_Write_AG_Register(CTRL_REG9, LSM9DS1_AG[CTRL_REG9] | 2);
		return tmp;
	}

	return 0;
}


int LSM9DS1_Set_AG_FIFO(FIFO_MODE_T mode, uint8_t threshold) {
	return LSM9DS1_Write_AG_Register(FIFO_CTRL, mode << 5 | (threshold & 0x1F));
}

int LSM9DS1_Set_AG_Reg1(G_ODR_T odr, G_FS_T fs, G_BW_T bw) {
	int ret = LSM9DS1_Write_AG_Register(CTRL_REG1_G, odr << 5 | fs << 3 | bw);
	if(ret) {
		set_g_divider(fs);
	}
	return ret;
}

int LSM9DS1_Set_G_ODR(G_ODR_T odr) {
	return LSM9DS1_Write_AG_Register(CTRL_REG1_G, (LSM9DS1_AG[CTRL_REG1_G] & 0x1F) | (odr << 5));
}

int LSM9DS1_Set_G_FS(G_FS_T fs) {
	int ret = LSM9DS1_Write_AG_Register(CTRL_REG1_G, (LSM9DS1_AG[CTRL_REG1_G] & 0xE7) | (fs << 3));
	if(ret) {
		set_g_divider(fs);
	}
	return ret;
}

int LSM9DS1_Set_G_BW(G_BW_T bw) {
	return LSM9DS1_Write_AG_Register(CTRL_REG1_G, (LSM9DS1_AG[CTRL_REG1_G] & 3) | bw);
}


int LSM9DS1_Set_AG_Interrupt1(INT1_CTRL_T value) {
	 return LSM9DS1_Write_AG_Register(INT1_CTRL, value);
}


/**
 * Sets CTRL_REG_3.
 * @param int Low-power mode enable.
 * @param int High-pass filter enable.
 * @param G_HPCF_T High-pass filter cut-off frequency.
 */
int LSM9DS1_Set_AG_Reg3(int lp_en, int hpf_en, G_HPCF_T hpcf) {
	return LSM9DS1_Write_AG_Register(CTRL_REG3_G, (lp_en & 1 << 7) | (hpf_en & 1 << 6) | hpcf);
}

/**
 * Turns on G+XL high-pass filter with the specified frequency.
 */
int LSM9DS1_Set_AG_HPF(G_HPCF_T hpcf) {
	return LSM9DS1_Write_AG_Register(CTRL_REG3_G, (LSM9DS1_AG[CTRL_REG3_G] & 0xF0) | hpcf | 0x40);
}

/**
 * Turns off the G+XL high-pass filter.
 */
int LSM9DS1_Disable_AG_HPF() {
	return LSM9DS1_Write_AG_Register(CTRL_REG3_G, LSM9DS1_AG[CTRL_REG3_G] & 0xBF);
}

/**
 * Set angular rate sensor sign and orientation register.
 * @param int Pitch axis (X) angular rate sign (0: positive sign; 1: negative sign)
 * @param int Roll axis (Y) angular rate sign (0: positive sign; 1: negative sign)
 * @param int Yaw axis (Z) angular rate sign (0: positive sign; 1: negative sign)
 * @param uint8_t (3-bit) Directional user orientation selection
 */
int LSM9DS1_Set_G_Orientation(int pitch_neg, int roll_neg, int yaw_neg, uint8_t orient) {
	return LSM9DS1_Write_AG_Register(ORIENT_CFG_G, (pitch_neg & 1 << 5) | (roll_neg & 1 << 4) | (yaw_neg & 1 << 3) | (orient & 7));
}


int16_t LSM9DS1_Get_AG_Temperature() {
	byte_t bytes[] = {0, 0};
	byte_str_t byteStr = {2, bytes};
	if(I2C_Cmd_Read_Bytes(LSM9DS1_AG_ADDR, OUT_TEMP_L, &byteStr) & 2) {
		return byteStr.bytes[0] | (byteStr.bytes[1] << 8);
	}
	return 0x8000; // output lowest negative number as error
}

int LSM9DS1_Get_G_Output(g_state_t *data) {
	return LSM9DS1_Get_Axes_Output(LSM9DS1_AG_ADDR, OUT_X_L_G, LSM9DS1_AG[CTRL_REG8] & 4, gDiv, (axes_state_t *) data);
}

int LSM9DS1_Get_XL_Output(axes_state_t *data) {
	return LSM9DS1_Get_Axes_Output(LSM9DS1_AG_ADDR, OUT_X_L_XL, LSM9DS1_AG[CTRL_REG8] & 4, xlDiv, data);
}

int LSM9DS1_Get_M_Output(axes_state_t *data) {
	/* TODO:
	 *   Verify that the auto increment actually works!
	 *   Otherwise the MSB of the sub-address must be 1 to enable it.
	 *
	 */
	return LSM9DS1_Get_Axes_Output(LSM9DS1_M_ADDR, OUT_X_L_M, 1, mDiv, data);
}


int LSM9DS1_Set_AG_Reg6(XL_ODR_T odr, XL_FS_T fs, int bw_sel, XL_BW_T bw) {
	int ret = LSM9DS1_Write_AG_Register(CTRL_REG6_XL, odr << 5 | fs << 3 | (bw_sel & 1 << 2) | bw);
	if(ret) {
		set_xl_divider(fs);
	}
	return ret;
}


/**
 * Set Control Register 1.
 * @param int Temperature compensation enable
 * @param OPERATIVE_MODE_T X and Y axes operative mode selection
 * @param M_ODR_T Output data rate selection
 * @param int FAST_ODR enables data rates higher than 80 Hz
 * @param int Self-test enable
 */
int LSM9DS1_Set_M_Reg1(int t_comp_en, OPERATIVE_MODE_T xy_op_mode, M_ODR_T odr, int fast_odr_en, int self_test_en) {
	return LSM9DS1_Write_M_Register(CTRL_REG1_M, (t_comp_en & 1 << 7) | (xy_op_mode << 5) | (odr << 2) | (fast_odr_en & 1 << 1) | (self_test_en & 1));
}

int LSM9DS1_Set_M_XY_Operative_Mode(OPERATIVE_MODE_T xy_op_mode) {
	return LSM9DS1_Write_M_Register(CTRL_REG1_M, (LSM9DS1_M[CTRL_REG1_M] & 0x9F) | (xy_op_mode << 5));
}

int LSM9DS1_Set_M_ODR(M_ODR_T odr) {
	return LSM9DS1_Write_M_Register(CTRL_REG1_M, (LSM9DS1_M[CTRL_REG1_M] & 0xE3) | (odr << 2));
}


/**
 * Set Control Register 2.
 * @param M_FS_T Full-scale configuration
 * @param int Reboot memory content (0: normal mode; 1: reboot memory content)
 * @param int Configuration registers and user register reset function. (0: default value; 1: reset operation)
 */
int LSM9DS1_Set_M_Reg2(M_FS_T fs, int reboot, int reset_op) {
	int ret = LSM9DS1_Write_M_Register(CTRL_REG2_M, (fs << 5) | (reboot & 1 << 3) | (reset_op & 1 << 2));
	if(ret) {
		set_m_divider(fs);
	}
	return ret;
}
int LSM9DS1_Set_M_FS(M_FS_T fs) {
	int ret = LSM9DS1_Write_M_Register(CTRL_REG2_M, (LSM9DS1_M[CTRL_REG2_M] & 0x9F) | (fs << 5));
	if(ret) {
		set_m_divider(fs);
	}
	return ret;
}

/**
 * Set Control Register 3.
 * @param int Disable I2C interface
 * @param int Low-power mode configuration
 * @param int SPI Serial Interface mode selection
 * @param M_OPERATING_MODE_T Operating mode selection
 */
int LSM9DS1_Set_M_Reg3(int disable_i2c, int low_power_en, int spi_rw, M_OPERATING_MODE_T op_mode) {
	return LSM9DS1_Write_M_Register(CTRL_REG3_M, (disable_i2c & 1 << 7) | (low_power_en & 1 << 5) | (spi_rw & 1 << 2) | op_mode);
}

int LSM9DS1_Set_M_Operating_Mode(M_OPERATING_MODE_T op_mode) {
	return LSM9DS1_Write_M_Register(CTRL_REG3_M, (LSM9DS1_M[CTRL_REG3_M] & 0xFC) | op_mode);
}

/**
 * Set Control Register 4
 * @param OPERATIVE_MODE Z-axis operative mode selection
 * @param int Big/Little Endian selection (default: 0)
 */
int LSM9DS1_Set_M_Reg4(OPERATIVE_MODE_T z_op_mode, int endianness) {
	return LSM9DS1_Write_M_Register(CTRL_REG4_M, (z_op_mode << 2) | (endianness & 1 << 1));
}

int LSM9DS1_Set_M_Z_Operative_Mode(OPERATIVE_MODE_T z_op_mode) {
	return LSM9DS1_Write_M_Register(CTRL_REG4_M, (LSM9DS1_M[CTRL_REG4_M] & 0xF3) | (z_op_mode << 2));
}
