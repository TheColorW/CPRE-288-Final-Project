 /*
 * main.c
 *
 *  Created on: Oct 29, 2026
 *      Author: carlos28
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ping.h"
#include "adc.h"
#include "uart.h"
#include "Timer.h"
#include <inc/tm4c123gh6pm.h>
#include "lcd.h"
#include "servo.h"
#include "button.h"
#include "open_interface.h"
#include "movement.h"
#include "i2c.h"
#include "bno055.h"
#include "init_imu.h"

#define SOUTH_HEADING   66.0f
#define HEADING_TOL     .4f


volatile int overflow_count;
volatile int flag;
volatile char uart_data;


void wait_for_calibration(void);
void turn_to_heading(oi_t *sensor_data, float target_deg);


int main(void) {
    timer_init();
    lcd_init();
    button_init();

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    // Initialize I2C1 + BNO055 and switch to 9-DOF sensor fusion
    init_imu();
    set_imu_mode(0x0C);

    // Step 1: Calibrate (hold still 2s for Gyro, then spin 360 deg for Mag)
    wait_for_calibration();

    float south_target = 180.0f;
    while (button_getButton() != 1) {
        bno055_convert_float_euler_h_deg(&south_target);
        lcd_printf("Point SOUTH &\npress SW1\nHeading: %.1f", south_target);
        timer_waitMillis(50);
    }
    timer_waitMillis(300);

    timer_waitMillis(3000);

    // Step 2: Rotate to face South
    turn_to_heading(sensor_data, south_target);


    // Step 3: Drive forward using your existing movement function
    double cm_driven = move_forward_until_stop(sensor_data, 250, 150);
    lcd_printf("Stopped at %.1f cm\nFlag: %d", cm_driven, flag);

    oi_free(sensor_data);
    return 0;
}


void wait_for_calibration(void){
    Calibration cal;


    while (1) {
        cal = calibrate_imu();
        lcd_printf("G:%d M:%d A:%d S:%d\nErr:%d",
                   cal.gyro, cal.magn, cal.accl, cal.syst, get_error());

        // Gyro + Mag = 3 gives reliable compass heading on flat ground
        if (cal.gyro == 3 && cal.magn == 3) {
            break;
        }

        // Safety override: press SW1 if indoor metal keeps Mag at 2
        if (cal.gyro == 3 && button_getButton() == 1) {
            break;
        }

        timer_waitMillis(100);
    }

    lcd_printf("IMU Ready!\nStarting turn...");
    timer_waitMillis(500);
}

void turn_to_heading(oi_t *sensor_data, float target_deg) {
    float heading = 0.0f;
    float error = 0.0f;
    int16_t speed = 0;

    while (1) {
        // Read fused heading directly (0.0 to 360.0 deg, no quaternion.c needed)
        bno055_convert_float_euler_h_deg(&heading);

        // Compute shortest signed angle error in range [-180, +180]
        error = target_deg - heading;
        while (error > 180.0f)  error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        lcd_printf("Target: %.1f\nCurr:   %.1f\nErr:    %.1f",
                   target_deg, heading, error);

        // Stop once inside tolerance band
        if (fabsf(error) <= HEADING_TOL) {
            break;
        }

        // Two-stage speed control: fast turn when far, slow creep near target
        speed = (fabsf(error) > 15.0f) ? 80 : 35;

        if (error > 0.0f) {
            // Need heading to increase -> turn clockwise
            oi_setWheels(-speed, speed);
        } else {
            // Need heading to decrease -> turn counterclockwise
            oi_setWheels(speed, -speed);
        }

        timer_waitMillis(15); // ~66 Hz update (BNO055 fuses at 100 Hz)
    }

    oi_setWheels(0, 0);
    timer_waitMillis(150); // Let chassis settle before driving forward
}

