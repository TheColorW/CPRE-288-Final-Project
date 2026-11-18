/*
*
*   uart.h
*
*   Created on: Oct 6, 2026
*   Updated on: Nov 18, 2026
*       Authors: carlos28, jfr72506, gm3vf
*/

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

// Volatiles for interrupts (DECLARE IN MAIN)
extern volatile char uart_data;
extern volatile char flag;

// Initialize
void uart_init(void);

// Transmit one char
void uart_sendChar(char data);

// Transmit full c-String
void uart_sendStr(const char *data);

// Receive one char
char uart_receive();

// Receive one char (nonblocking)
char uart_receive_nonblock(char *data);

// Initialize interrupts
void uart_interrupt_init();

// Interrupt handler
void uart_interrupt_handler();

#endif /* UART_H_ */
