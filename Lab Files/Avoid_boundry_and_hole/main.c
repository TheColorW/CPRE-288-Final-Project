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

volatile int overflow_count;
volatile int flag;
volatile char uart_data;

int main(void) {
    timer_init();
    servo_init();
    lcd_init();
    button_init();
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);


    double cm_driven = move_forward_until_stop(sensor_data, 250, 150);

    lcd_printf("Flag hit at %.lf\n because of Flag:%d",cm_driven, flag);

    oi_free(sensor_data);

//    volatile int flag;
//
//
//    int turnDir = 0; // 0 = counterclockwise, 1 = clockwise
//    float currentDegree = 90.0; // used for logic with sw1 and sw2 (90 is default)
//    char message[20] = "counterclockwise"; // initial direction message
//    servo_move(currentDegree); //initial degree to start
//
//    while(1)
//        {
//
//            if(flag == 0){
//                oi_update(sensor_data);
//
//            }
//
//            int leftColorSensor = sensor_data->cliffLeftSignal;
//            int frontLeftColorSensor = sensor_data->cliffFrontLeftSignal;
//            int rightColorSensor = sensor_data->cliffRightSignal;
//            int frontRightColorSensor = sensor_data->cliffFrontRightSignal;
//            //50 was black, 2500 was grey, and 2800 was white... rough estimates based on returned values
//            //cybot1 grey: 1700 - 2000, White: 2650 - 2800
//            lcd_printf("Left: %d\nRight: %d", frontLeftColorSensor, frontRightColorSensor);
//
//
//            //Cybot is directly above border
//             if((frontRightColorSensor > 2650 || frontRightColorSensor < 300) && (frontLeftColorSensor > 2650 || frontLeftColorSensor < 300)) {
//                 lcd_printf("first- go back and turn counter");
//                 move_backward(sensor_data, 10, 150);
//                 turn_anticlockwise(sensor_data, 180);
//             } else if(frontRightColorSensor > 2650 || frontRightColorSensor < 300) {
//                 lcd_printf("second- turn counter");
//                 turn_anticlockwise(sensor_data, 10);
//             } else if(frontLeftColorSensor > 2650 || frontLeftColorSensor < 300) {
//                 lcd_printf("third- turnr");
//                 turn_clockwise(sensor_data, 10);
//             }
//
//             lcd_printf("forward?");
//
//             double forwared = move_forward(sensor_data, 10, 150);
//             oi_setWheels(0,0);
//             lcd_printf("rescan with this val traveled %lf\n", forwared );
//             oi_free(sensor_data);
//             oi_update(sensor_data);
//
//        }




    //servo_move(265.0); // 265 degrees for some reason is 180 w/ 1 at end of actual seconds
                     // 0 is not 0, stops at ~30 to 45 degrees

    //actual 0 is -50, actual 180 is ~265, actual 90 is 115 for cybot 01 (equation for cybot 1: y = 0.0097x + 0.7361)
}
