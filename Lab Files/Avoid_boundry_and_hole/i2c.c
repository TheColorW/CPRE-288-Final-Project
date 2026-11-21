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
#include "bno055.h"


void I2C_init(void){
    SYSCTL_RCGCGPIO_R |= 0x01;
    SYSCTL_RCGCI2C_R = 0x02;
    timer_waitMillis(1);                 // Small delay before accessing device after turning on clock

    GPIO_PORTA_AFSEL_R |= 0xC0;
    GPIO_PORTA_PCTL_R &= 0x00FFFFFF;     // Force 0's in the disired locations
    GPIO_PORTA_PCTL_R |= 0x33000000;           // Force 1's in the disired locations
    GPIO_PORTA_DEN_R |= 0xC0;
    GPIO_PORTA_ODR_R |= 0x80;

    I2C1_MCR_R = 0x10;
    I2C1_MTPR_R = 0x13;

    //To initialize the GPIO for the rst and addr

    SYSCTL_RCGCGPIO_R |= 0x02;
    timer_waitMillis(1);

    GPIO_PORTB_DIR_R  |= 0xC0;   // PB6 and PB7 as outputs (0b1100_0000)
    GPIO_PORTB_DEN_R  |= 0xC0;   // Digital enable PB6 and PB7

    GPIO_PORTB_DATA_R &= ~0x80;  // PB7 = 0 -> Sets BNO055 I2C address to 0x28
    GPIO_PORTB_DATA_R |= 0x40;   // PB6 = 1 -> Releases active-low Reset (RSTN)

}
static int I2C1_Wait_Done(void){
    while(I2C1_MCS_R & 1);
    return I2C1_MCS_R & 0xE;
}

int I2C1_Write(uint8_t sAddr, uint8_t rAddr, const uint8_t *data, uint8_t count){
    uint8_t error, i;

    if (count == 0) return 0;

    I2C1_MSA_R = sAddr <<1;
    I2C1_MDR_R = rAddr;
    I2C1_MCS_R = 0x3;

    error = I2C1_Wait_Done();
    if(error){
        I2C1_MCS_R = 0x04;           // STOP on error
        return -1;
    }

    //--------------------------------------------------------

    for (i =0; i<count; i++){
        I2C1_MDR_R = data[i];
        if(i==count -1){
            I2C1_MCS_R = 0x05;
        }else{
            I2C1_MCS_R = 0x01;
        }
        error = I2C1_Wait_Done();
        if(error){
            I2C1_MCS_R = 0x04;
            return -1;
        }
    }


    while (I2C1_MCS_R & 0x40);       // Wait until BUSBSY clears
    return 0;
}

int I2C1_Read(int sAddr, uint8_t rAddr, uint8_t *data, uint8_t count){
    uint8_t error, i;


    if(count == 0) return 0;

    I2C1_MSA_R = sAddr << 1;
    I2C1_MDR_R = rAddr;
    I2C1_MCS_R = 0x3;
    error = I2C1_Wait_Done();
    if(error){
        I2C1_MCS_R = 0x04;
        return -1;
        }
    I2C1_MSA_R = (sAddr << 1) | 1;
    
    if(count == 1){
        I2C1_MCS_R = 0x07; //what exactly does this do?
        error = I2C1_Wait_Done();
        if(error) return -1;

        data[0] = (uint8_t)(I2C1_MDR_R & 0xFF);
        error = I2C1_Wait_Done();
    }else {
        for (i = 0; i < count; i++) {
            if (i == 0) {
                I2C1_MCS_R = 0x0B;  
            } else if (i == count - 1) {
                I2C1_MCS_R = 0x05;   
            } else {
                I2C1_MCS_R = 0x09; 
            }

            error = I2C1_Wait_Done();
            if (error) {
                if (i < count - 1) {
                    I2C1_MCS_R = 0x04; // Force STOP if aborted mid-burst
                }
                return -1;
            }
            data[i] = (uint8_t)(I2C1_MDR_R & 0xFF);
        }
    }

        while(I2C1_MCS_R &0x40);
        return 0; 
        
}
