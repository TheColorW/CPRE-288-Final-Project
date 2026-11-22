/*
 * uart.h
 *
 *  Created on: Oct 6, 2026
 *      Author: carlos28
 */

#ifndef i2c_H_
#define i2c_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "bno055.h"

void I2C_init(void);


int I2C1_Write(uint8_t sAddr, uint8_t rAddr, const uint8_t *data, uint8_t count);

int I2C1_Read(int sAddr, uint8_t rAddr, uint8_t *data, uint8_t count);

#endif /* UART_H_ */
