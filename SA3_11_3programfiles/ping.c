/*
 * ping.c
 *
 *  Created on: Oct 28, 2026
 *      Author: carlos28
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ping.h"
#include "Timer.h"
#include <inc/tm4c123gh6pm.h>

volatile int rising_time;
volatile int end_time;
volatile int capture_state;
volatile int overflow_count;



void TIMER3B_handler() {
    // Handles ping interrupts
    if (TIMER3_MIS_R & 0x0400) {//checking if the interrupt was triggered by the timer 3b capture
        TIMER3_ICR_R = 0x0400;

        if (capture_state == 1) {
            // First edge: Rising edge timestamp
            rising_time = TIMER3_TBR_R & 0x00FFFFFF;
            capture_state = 2;
        } else if (capture_state == 2) {
            // Second edge: Falling edge timestamp
            end_time = TIMER3_TBR_R & 0x00FFFFFF;
            capture_state = 3;
            TIMER3_IMR_R &= 0x0400; // Mask interrupt until next reading
        }
    }
}

void ping_init() {
    //write the initialization of registers and timer
    SYSCTL_RCGCTIMER_R |= 0x8; //enable timer 3 clock
    SYSCTL_RCGCGPIO_R |= 0x2; //enable port b clock
    timer_waitMillis(3);

    GPIO_PORTB_DIR_R |= 0x08; //pin 3 set as an output
    GPIO_PORTB_AFSEL_R &= 0xF7; //Disable alternate function
    GPIO_PORTB_PCTL_R &= ~0xF000; //Clear pin 3 configuration
    GPIO_PORTB_PCTL_R |= 0x7000; //pin 3 config (unsure about this one)
    GPIO_PORTB_DEN_R |= 0x08; //pin 3 set to digital

    TIMER3_CTL_R &= ~0x100; //disable timer3b for setup
    TIMER3_CFG_R = 0x4; //timers are split into 2 16-bit timers
    TIMER3_TBMR_R = 0x07; //timer 3b mode is set (count down, edge-time, capture)
    TIMER3_CTL_R |= 0xC00; //both edges trigger timer 3b interrupt (im pretty sure this is correct???)
    TIMER3_TBILR_R = 0xFFFF; //timer loaded with initial count down num
    TIMER3_TBPR_R = 0xFF; //timer 3b pre-scaler set
    TIMER3_IMR_R |= 0x400; //interrupt mask set for timer 3b
    //TIMER3_CTL_R |= 0x100; //enable the timer 3b
    NVIC_EN1_R |= 0x10;
    IntRegister(INT_TIMER3B, TIMER3B_handler); //Handler not coded yet



}

int ping_read() { 
    //read the ping values like adc (for part 2)
    //Timer already masked -- when does this happen? - TIMER3_IMR_R &= ~0x0400;
   //GPIO_PORTB_AFSEL_R &= 0xF7; // Preserve all other bits while setting pin 3 to GPIO mode
   //TIMER3_IMR_R &= 0x0400;
   //Configure pin 3 as output

   TIMER3_CTL_R &= ~0x100;
   GPIO_PORTB_AFSEL_R &= 0xF7;
   GPIO_PORTB_DIR_R |= 0x08;
  // GPIO_PORTB_DATA_R &= ~0x08;
   //timer_waitMicros(2);
   GPIO_PORTB_DATA_R |= 0x08; //Set pin 3 high
   timer_waitMicros(5);
   GPIO_PORTB_DATA_R &= ~0x08; //Set pin 3 low

   //Configure pin 3 to input

   GPIO_PORTB_AFSEL_R |= 0x08; //Preserve all other bits while setting pin 3 to AFSEL mode
   GPIO_PORTB_DIR_R &= ~0x08; //Set pin 3 as input
   //GPIO_PORTB_PCTL_R &= ~0xF000;
   //GPIO_PORTB_PCTL_R |= 0x7000;


   //Capture mode after sending input trigger
   capture_state = 1;
   //Clear interrupt timer
   TIMER3_ICR_R = 0x0400;  //Got from slides, not sure why its here
   TIMER3_CTL_R |= 0x100;
   //Un-mask interrupt timer
  // TIMER3_IMR_R &= ~0x0400;

   while(capture_state != 3){}

  int amount_ticks = 0;
  if(rising_time >= end_time){
      amount_ticks = rising_time - end_time;
  }else{
    amount_ticks = rising_time + 0x01000000 - end_time;
    overflow_count++;
  }

  return amount_ticks;

   //Catch rising edge, store TIMER3_TBR_R in var
   // int rising = TIMER3_TBR_R //Defiantly not exactly correct

   //Catch falling edge, store TIMER3_IMR_R in var
   // int end = TIMER3_TBR_R //Defiantly not exactly correct

   // Return difference
   // return (start - end) * delta?; //This could be wrong
}
