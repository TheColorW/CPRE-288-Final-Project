/*
 * adc.c
 *
 *  Created on: Oct 21, 2026
 *      Author: carlos28
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "adc.h"
#include "Timer.h"
#include <inc/tm4c123gh6pm.h>


void adc_init() {
    SYSCTL_RCGCGPIO_R |= 0b10; //enables and provides clock to port b
    SYSCTL_RCGCADC_R |= 0x1; //enables and provides clock to module 0
    timer_waitMillis(3);

    GPIO_PORTB_AFSEL_R |= 0x10; // pb4 is controlled by alternate hardware function
    GPIO_PORTB_DEN_R &= ~0x10; //forces input to be received as analog
    GPIO_PORTB_AMSEL_R |= 0x10; //analog mode for port b4 is enabled
    GPIO_PORTB_ADCCTL_R = 0x00; //ensuring this is set to its default

    // sample sequencer config

    ADC0_ACTSS_R = 0x0; //sample sequencer disabled for programming
    ADC0_EMUX_R = 0x0; //default trigger selected, will be using SS0

    ADC0_SSMUX0_R = 0xA; //input source is AIN10 for all samples
    ADC0_SSCTL0_R = 0x6; // raw interrupt samples

    ADC0_IM_R = 0x1; //something to do with interrupt masks
    ADC0_ACTSS_R = 0x1; //sample sequencer re-enabled

    // averaging mechanism for part 2
    ADC0_SAC_R = 0x4; //sample averaging is set to 16x hardware oversampling (this reduces variability)
}

int adc_read() {

    //takes a sample
    ADC0_PSSI_R = 0x01; //triggers start of conversion

    while ((~ADC0_RIS_R & 0x01)) {
        //wait for adc to complete conversions
    }
    ADC0_ISC_R = 0x01;

    int value = ADC0_SSFIFO0_R;

    return value; //should return converted value (raw ir)
}
