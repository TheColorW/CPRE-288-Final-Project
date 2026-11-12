/*
 * servo.c
 *
 *  Created on: Nov 3, 2026
 *      Author: carlos28
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "servo.h"
#include "Timer.h"
#include <inc/tm4c123gh6pm.h>

void servo_init() {
    SYSCTL_RCGCTIMER_R |= 0x2; //enable timer 1 clock
    SYSCTL_RCGCGPIO_R |= 0x2; //enable port b clock
    timer_waitMillis(3);

    GPIO_PORTB_DIR_R |= 0x20; //pin 5 set as an output
    GPIO_PORTB_AFSEL_R |= 0x20; //enable alternate function
    GPIO_PORTB_PCTL_R &= ~0xF00000; //Clear pin 5 configuration
    GPIO_PORTB_PCTL_R |= 0x700000; //pin 5 config
    GPIO_PORTB_DEN_R |= 0x20; //pin 5 set to digital

    TIMER1_CTL_R &= ~0x100; //disable timer1b for setup
    TIMER1_CFG_R = 0x4; //timers are split into 2 16-bit timers
    TIMER1_TBMR_R = 0xA; //timer 1b mode is set (count down, PWM, Periodic)
    TIMER1_CTL_R &= 0x0000; //output level is set to not inverted
    TIMER1_TBPR_R = 0x04; //timer 1b pre-scaler set
    TIMER1_TBILR_R = 0xE200; //timer loaded with initial count down num (320000) these 2 together make the period
    TIMER1_TBPMR_R = 0x04; //initial match values set
    TIMER1_TBMATCHR_R = 0xE200; //initial match values set
    TIMER1_CTL_R |= 0x100; //enable timer 1b
}

int servo_move(float degrees) {
    //(equation for cybot 1: y = 0.0097x + 0.7361)
    //equation for cybot 28: y = 0.0096x + 0.4
    //equation for cybot 4: y = 0.0094x + 0.4944

    float actualMS = (0.0094 * degrees) + 0.4944; //converting to real ms needed

    float pulse_width = 16000.0 * actualMS; //length of high
    float falling_edge_time = 320000.0 - pulse_width; //length of low

    int match_register = (int) falling_edge_time;

    //print statements for debugging
    printf("%f\n", actualMS);
    printf("%f\n", falling_edge_time);

    //this sets the match registers up to the correct value based on falling edge time
    TIMER1_TBPMR_R = (match_register >> 16) & 0xFF;
    TIMER1_TBMATCHR_R = match_register & 0xFFFF;

    return match_register;
}


