/*
 * i2c.h
 *
 *  Created on: 28 Jan 2020
 *      Author: TDP4 Team 3
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "chip.h"

#define CLK_100KHZ 100000 // Standard mode
#define CLK_400KHZ 400000 // Fast mode
#define CLK_1MHZ 1000000  // Fast mode Plus

#define DEFAULT_CLK CLK_100KHZ

/**
 * Data types
 */

typedef uint8_t byte_t;

typedef struct byte_string {
	int size;
	byte_t *bytes;
} byte_str_t;

/**
 * Function definitions
 */

void Init_I2C();

void Deinit_I2C();

int I2C_Send_Byte(byte_t, byte_t);

int I2C_Send_Bytes(byte_t, byte_str_t *);

int I2C_Cmd_Read_Bytes(byte_t, byte_t, byte_str_t *);

#endif /* INC_I2C_H_ */