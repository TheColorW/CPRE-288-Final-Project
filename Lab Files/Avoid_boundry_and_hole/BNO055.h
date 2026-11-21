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
#include "driverlib/interrupt.h"

void I2C_init(void);

static int I2C1_Wait_Done(void)

int I2C1_Write(int sAddr, int rAddr, int data)

void I2C1_Read(int sAddr, uint8_t rAddr)

#endif /* UART_H_ */
