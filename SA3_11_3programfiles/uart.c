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
#include "uart.h"
#include "timer.h"


void uart_init(){
    SYSCTL_RCGCGPIO_R |= 0b10;
    SYSCTL_RCGCUART_R = 0x02;
    timer_waitMillis(1);                 // Small delay before accessing device after turning on clock

    GPIO_PORTB_AFSEL_R |= 0x03;
    GPIO_PORTB_PCTL_R &= 0xFFFFFF00;     // Force 0's in the disired locations
    GPIO_PORTB_PCTL_R |= 0x11;           // Force 1's in the disired locations
    GPIO_PORTB_DEN_R |= 0x03;
    GPIO_PORTB_DIR_R &= ~0x03;           // Force 0's in the disired locations
    GPIO_PORTB_DIR_R |= 0b10;            // Force 1's in the disired locataions

    UART1_CTL_R &= 0xFFFFFFFE;           // disables the uart as we set it up below

    UART1_IBRD_R = 0x08;
    UART1_FBRD_R = 0x2C;
    UART1_LCRH_R = 0x60;
    UART1_CC_R = 0x0;

    UART1_CTL_R |= 0x1;                  // re-enables the uart
}

void uart_sendChar(char data){

    while((UART1_FR_R & 0x20) != 0){
        //nothing is actually here, just waits while FIFO is full. Then it sends data.
    }

    UART1_DR_R = data;
}

char uart_receive(){
    char data = 0;

    while((UART1_FR_R & 0x10) != 0){
        // nothing here, waits while FIFO is empty
    }

    data = (char)(UART1_DR_R & 0xFF);

    return data;
}

void uart_sendStr(const char *data){//this loops through the string to send it
    int i = 0;
    while(data[i] != '\0'){
        uart_sendChar(data[i]);
        i++;
    }
}

void uart_interrupt_init(){

    // Enable interrupts for receiving bytes through UART1
    UART1_IM_R |= 0x10; //enable interrupt on receive - page 924

    // Find the NVIC enable register and bit responsible for UART1 in table 2-9
    // Note: NVIC register descriptions are found in chapter 3.4
    NVIC_EN0_R |= 0x40; //enable uart1 interrupts - page 104

    // Find the vector number of UART1 in table 2-9 ! UART1 is 22 from vector number page 104
    IntRegister(INT_UART1, uart_interrupt_handler); //give the microcontroller the address of our interrupt handler - page 104 22 is the vector number

}

void uart_interrupt_handler(){

    // STEP1: Check the Masked Interrup Status

    //STEP2:  Copy the data

    //STEP3:  Clear the interrup

    //this code below is partially from slides, there is half that I did not include
    //because the lab says to only set up for receiving characters, not sending. lmk if im wrong pls
    if((UART1_MIS_R & 0x10) != 0){ //step1

        uart_data = uart_receive(); //step2
        flag = 1;

        UART1_ICR_R = 0x10; //step3

    }

}

int uart_receive_nonblock(char *data){
    if(UART1_FR_R & 0x10){
        return 0;
    }

    *data = (char)(UART1_DR_R & 0xFF);
    return 1;
}
