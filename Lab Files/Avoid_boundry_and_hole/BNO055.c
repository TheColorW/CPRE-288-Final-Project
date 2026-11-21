/*
 * uart.c
 *
 *  Created on: Oct 6, 2026
 *      Author: carlos28
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inc/tm4c123gh6pm.h>
#include "i2c.h"
#include "timer.h"


void i2c_init(){
    SYSCTL_RCGCGPIO_R |= 0x01;
    SYSCTL_RCGCI2C_R = 0x02;
    timer_waitMillis(1);                 // Small delay before accessing device after turning on clock

    GPIO_PORTA_AFSEL_R |= 0xC0;
    GPIO_PORTA_PCTL_R &= 0x00FFFFFF;     // Force 0's in the disired locations
    GPIO_PORTA_PCTL_R |= 0x33000000;           // Force 1's in the disired locations
    GPIO_PORTA_DEN_R |= 0xC0;
    GPIO_PORTA_ODR_R |= 0x80;

    I2C1_MCR_R = 0x10;
    I2C1_MTPR_R = 0x7;

}
static int I2C1_Wait_Done(void){
    while(I2C1_MCS_R & 1);
    return I2C1_MCS_R & 0xE;
}

int I2C1_Write(int sAddr, int rAddr, int data){
    uint8_t error;

    I2C1_MSA_R = sAddr <<1;
    I2C_MDR_R = rAddr;
    I2C1_MCS_R = 0x3;

    error = I2C1_Wait_Done();
    if(error){return error;}
    I2C1_MDR_R = data;
    I2C1_MCS_R = 0x5;
    error = I2C1_Wait_Done();
    while(I2C1_MCS_R & 0x40);
    error = I2C1_MCS_R & 0xE;
    return error;
}

void I2C1_Read(int sAddr, uint8_t rAddr){
    uint8_t error, data;
    while(1){
        I2C1_MSA_R = sAddr << 1;
        I2C1_MDR_R = rAddr;
        I2C_MCS_R = 0x3; 
        error = I2C1_Wait_Done();
        if(error){//GPIO_PORTF_DATA_R |= 0x0F;
            }
        data = I2C1_MDR_R;
        while(I2C_MCS_R &0x40);
        error = I2C1_MCS_R & 0xE;
        if(error){//GPIO_PORTF_DATA_R |= 0x0F;
            }
        GPIO_PORTB_DATA_R = data;
        
    }
}
