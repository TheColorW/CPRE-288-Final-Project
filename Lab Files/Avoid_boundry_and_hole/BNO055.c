/*
 * uart.c
 *
 *  Created on: Oct 6, 2026
 *      Author: carlos28
 * @note    Adapted from the "bno055-to-tiva" library by Michael Rencheck (rencheckyoself).
 *          Source: https://github.com/rencheckyoself/bno055-to-tiva
 *          Commit: 27a89eb (Accessed November 21, 2026)
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inc/tm4c123gh6pm.h>
#include "BNO055.h"
#include "timer.h"
#include "i2c.h"

static s8 error;
static struct bn055_euler_float_t last_pos;

s8 _imu_i2c_read(u8 dev_address, u8 reg_address, u8 *arr_data, u8 count) {
    return (s8)I2C1_ReadBytes(dev_address, reg_address, arr_data, count);
}

s8 _imu_i2c_write(u8 dev_address, u8 reg_address, u8 *var_data, u8 count) {
    return (s8)I2C1_WriteBytes(dev_address, reg_address, var_data, count);
}


void ms_delay(u32 ms)
{
    timer_waitMillis(ms);
}

void init_imu(void) {
    static struct bno055_t sensor;

    i2c_init();

    sensor.bus_write  = _imu_i2c_write;
    sensor.bus_read   = _imu_i2c_read;
    sensor.delay_msec = ms_delay;
    sensor.dev_addr   = BNO055_I2C_ADDR1; // 0x28 (or BNO055_I2C_ADDR2 = 0x29 if COM3 pin is HIGH)

    bno055_init(&sensor);
}

// void set_imu_mode(u8 op_mode)
// {
//   error = bno055_set_operation_mode(op_mode);
// }

// struct bno055_euler_float_t get_abs_position()
// {
//   struct bno055_quaternion_t q; // the raw data from the sensor
//   Quaternion qf; // cast the raw data from int into float

//   error += bno055_read_quaternion_wxyz(&q); // read the sensor

//   qf = bnoquat_to_float(&q); // convert raw data to a float

//   scale_divide(&qf, QUATERNION_SCALING); // scale the raw data per the sensor data sheet

//   last_pos = toEuler(&qf); // calculate the euler angles

//   return last_pos;
// }

// Calibration calibrate_imu()
// {
//   u8 gyro_cal = 0, acc_cal = 0, mag_cal = 0, sys_cal = 0;

//   error = bno055_get_gyro_calib_stat(&gyro_cal);
//   error += bno055_get_mag_calib_stat(&mag_cal);
//   error += bno055_get_accel_calib_stat(&acc_cal);
//   error += bno055_get_sys_calib_stat(&sys_cal);

//   Calibration output;
//   output.gyro = gyro_cal;
//   output.accl = acc_cal;
//   output.magn = mag_cal;
//   output.syst = sys_cal;

//   return output;
// }

// s8 get_error()
// {
//   return error;
// }