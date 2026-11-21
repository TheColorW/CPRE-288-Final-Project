/*
 * uart.h
 *
 *  Created on: Oct 6, 2026
 *      Author: carlos28
 */

#ifndef BNO055_H_
#define BNO055_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"


typedef struct Calibration
{
/// \brief store all calibration values
/// \returns a struct with the calibration data
    unsigned int gyro;
    unsigned int accl;
    unsigned int magn;
    unsigned int syst;
} Calibration;


int8_t _imu_i2c_read(uint8_t dev_address, uint8_t reg_address, uint8_t *arr_data, uint8_t count);

int8_t _imu_i2c_write(uint8_t dev_address, uint8_t reg_address, uint8_t *var_data, uint8_t count);

void ms_delay(uint32_t ms);

void init_imu(void);
//struct bno055_euler_float_t get_abs_position();

Calibration calibrate_imu();

#endif /* UART_H_ */
