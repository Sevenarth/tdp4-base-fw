/*
 * LSM9DS1.c
 *
 *  Created on: 28 Jan 2020
 *      Author: TDP4 Team 3
 */

#include "LSM9DS1.h"
#include "i2c.h"

int LSM9DS1_Write_AG_Register(byte_t addr, byte_t value) {
	byte_str_t req = {2, {addr, value}};
	if(I2C_Send_Bytes(LSM9DS1_AG_ADDR, &req) & 2) {
		LSM9DS1_AG[addr] = value; // update local register
		return 2;
	}

	return 0;
}

int LSM9DS1_Reset_FIFO() {
	if(LSM9DS1_AG[FIFO_CTRL] & 0x20) {
		int ret;

		byte_str_t req = {2, {FIFO_CTRL, 0}};

		ret = I2C_Send_Bytes(LSM9DS1_AG_ADDR, &req);

		req.bytes[1] = LSM9DS1_AG[FIFO_CTRL]; // restore previous register data
		ret &= I2C_Send_Bytes(LSM9DS1_AG_ADDR, &req);

		return ret & 2; // returns 2 if successful, otherwise 0
	}

	return 0;
}

int LSM9DS1_Enable_AG_FIFO() {
	if(!(LSM9DS1_AG[FIFO_CTRL] & 0x20)) {
		return LSM9DS1_Write_AG_Register(FIFO_CTRL, (LSM9DS1_AG[FIFO_CTRL] & 0x1F) | 0x20);
	}

	return 0;
}


int LSM9DS1_Set_AG_FIFO(FIFO_MODE_T mode, uint8_t threshold) {
	return LSM9DS1_Write_AG_Register(FIFO_CTRL, mode << 5 | (threshold & 0x1F));
}


int LSM9DS1_Set_AG_Reg1(G_ODR_T odr, G_FS_T fs, G_BW_T bw) {
	return LSM9DS1_Write_AG_Register(CTRL_REG1_G, odr << 5 | fs << 3 | bw);
}

int LSM9DS1_Set_G_ODR(G_ODR_T odr) {
	return LSM9DS1_Write_AG_Register(CTRL_REG1_G, (LSM9DS1_AG[CTRL_REG1_G] & 0x1F) | (odr << 5));
}

int LSM9DS1_Set_G_FS(G_FS_T fs) {
	return LSM9DS1_Write_AG_Register(CTRL_REG1_G, (LSM9DS1_AG[CTRL_REG1_G] & 0xE7) | (fs << 3));
}

int LSM9DS1_Set_G_BW(G_BW_T bw) {
	return LSM9DS1_Write_AG_Register(CTRL_REG1_G, (LSM9DS1_AG[CTRL_REG1_G] & 3) | bw);
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
	return LSM9DS1_Write_AG_Register(ORIENT_CFG_G, pitch_neg & 1 << 5 | roll_neg & 1 << 4 | yaw_neg & 1 << 3 | orient & 7);
}


int16_t LSM9DS1_Get_AG_Temperature() {
	byte_str_t byteStr = {2, {0, 0}};
	if(I2C_Cmd_Read_Bytes(LSM9DS1_AG_ADDR, OUT_TEMP_L, &byteStr) & 2) {
		return byteStr.bytes[0] | (byteStr.bytes[1] << 8);
	}
	return 0x8000; // output lowest negative number as error
}

int16_t LSM9DS1_Get_Axes_Output(byte_t addr, byte_t start_reg, int auto_increment) {
	int16_t axes[] = {0, 0, 0};

	if(auto_increment) { // auto increment turned on
		byte_str_t byteStr = {6, {0, 0, 0, 0, 0, 0}};
		if(I2C_Cmd_Read_Bytes(addr, start_reg, &byteStr) == 6) {
			axes[0] = byteStr.bytes[0] | (byteStr.bytes[1] << 8);
			axes[1] = byteStr.bytes[2] | (byteStr.bytes[3] << 8);
			axes[2] = byteStr.bytes[4] | (byteStr.bytes[5] << 8);
		}
	} else {
		byte_str_t byteStr = {2, {0, 0}};
		for(int i = 0; i < 3; i++) {
			if(I2C_Cmd_Read_Bytes(addr, start_reg+(2*i), &byteStr) & 2) {
				axes[i] = byteStr.bytes[0] | (byteStr.bytes[1] << 8);
			}
		}
	}

	return axes;
}

g_state_t LSM9DS1_Get_G_Output() {
	return *(g_state_t *)(LSM9DS1_Get_Axes_Output(LSM9DS1_AG_ADDR, OUT_X_L_G, LSM9DS1_AG[CTRL_REG8] & 4));
}

axes_state_t LSM9DS1_Get_XL_Output() {
	return *(axes_state_t *)(LSM9DS1_Get_Axes_Output(LSM9DS1_AG_ADDR, OUT_X_L_XL, LSM9DS1_AG[CTRL_REG8] & 4));
}

axes_state_t LSM9DS1_Get_M_Output() {
	/* TODO:
	 *   Verify that the auto increment actually works!
	 *   Otherwise the MSB of the sub-address must be 1 to enable it.
	 */
	return *(axes_state_t *)(LSM9DS1_Get_Axes_Output(LSM9DS1_M_ADDR, OUT_X_L_M, 1));
}
