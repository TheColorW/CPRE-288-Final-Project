/*
 * adc.c
 *
 *  Created on: Nov 2, 2026
 *      Author: gm3vf
 */

#include "adc.h"
#include <inc/tm4c123gh6pm.h>
#include <stdint.h>

// I made the choice to use ADC0 and sequencer 3 but we can change that if we want.
// we also have to use AIN10 on PB4 and the clock is the 16 MHz clock
void adc_init(void)
{

    // first we will enable the clock for port B and ADC0
    SYSCTL_RCGCGPIO_R |= 0X02;
    SYSCTL_RCGCADC_R |= 0X01;

    while ((SYSCTL_PRGPIO_R & 0X02) == 0)
    { // WAIT UNTIL READY

    }

    while ((SYSCTL_PRADC_R & 0X01) == 0)
    { // WAIT UNTIL READY

    }

    /* NOW WE WILL CONFIGURE THE ANALOG INPUT FOR PB4
     see one note for explination on how they each work
     */
    GPIO_PORTB_DIR_R &= ~0x10;
    GPIO_PORTB_AFSEL_R |= 0x10;
    GPIO_PORTB_DEN_R &= ~0x10;
    GPIO_PORTB_AMSEL_R |= 0x10;
    GPIO_PORTB_PUR_R &= ~0x10;
    GPIO_PORTB_PDR_R &= ~0x10;

    ADC0_ACTSS_R &= ~0x08;
    ADC0_CC_R &= ~0x0F;
    ADC0_PC_R = (ADC0_PC_R & ~0x0F) | 0x01;
    ADC0_SAC_R &= ~0x07;
    ADC0_EMUX_R &= ~0xF000;
    ADC0_SSMUX3_R = (ADC0_SSMUX3_R & ~0x0F) | 0x0A;
    ADC0_SSCTL3_R = (ADC0_SSCTL3_R & ~0x0F) | 0x06;
    ADC0_SSOP3_R &= ~0x01;
    ADC0_IM_R &= ~0x08;
    ADC0_ISC_R = 0x08;
    ADC0_ACTSS_R |= 0x08;

}

int adc_read(void)
{
    int raw;
    ADC0_PSSI_R = 0x08;
    while ((ADC0_RIS_R & 0x08) == 0)
    {
        raw = (int) (ADC0_SSFIFO3_R & 0x0FFF);
        ADC0_ISC_R = 0x08;

    }
    return raw;

}
