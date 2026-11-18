/*
 *   uart.c
 *
 *   Created on: Oct 6, 2026
 *   Updated on: Nov 18, 2026
 *      Authors: carlos28, jfr72506, gm3vf
 */


#include "timer.h"
#include "uart.h"

// Initialize UART
void uart_init(void)
{
    SYSCTL_RCGCGPIO_R |= 0x02;  // Enable Port B Clock
    SYSCTL_RCGCUART_R |= 0x02;  // Enable Port B UART Clock
    timer_waitMillis(1); // Small delay before accessing device after turning on clock

    GPIO_PORTB_AFSEL_R |= 0x03;  // Set PB0 and PB1 to peripheral
    GPIO_PORTB_PCTL_R = 0x11;   // Set PB0 to U1RX and PB1 to U1TX
    GPIO_PORTB_DEN_R = 0x03;    // Enable PB0 and PB1
    GPIO_PORTB_DIR_R = 0x02;    // Set PB0 as output PB1 as input

    UART1_CTL_R &= ~0x01;   // Disable UART1 for config

    UART1_IBRD_R = 0x08; // Write int portion of BRD to IBRD
    UART1_FBRD_R = 0x2C;   // Write fractional portion of BRD to FBRD
    UART1_LCRH_R = 0x60; // Write serial communication parameters (page 916) * 8bit and no parity
    UART1_CC_R = 0x0;   // Use system clock as clock source (page 939)

    UART1_CTL_R |= 0x01;    // Re-enable UART1
}

// Transmit one char
void uart_sendChar(char data)
{
    while ((UART1_FR_R & 0x20))
    {
        // Wait while FIFO is full
    }

    UART1_DR_R = data;  // Transmit data
}

// Transmit full c-String
void uart_sendStr(const char *data)
{
    int i = 0;
    while (data[i] != '\0')
    {
        uart_sendChar(data[i]);
        i++;
    }
}

// Receive one char
char uart_receive()
{
    char data = 0;

    while (UART1_FR_R & 0x10)
    {
        // Wait while FIFO is empty
    }

    data = (char) (UART1_DR_R & 0xFF);

    return data;
}

// Receive one char (nonblocking)
char uart_receive_nonblock(char *data)
{
    if (UART1_FR_R & 0x10)
    {
        return 0;
    }

    *data = (char) (UART1_DR_R & 0xFF);
    return 1;
}

// Initialize interrupts
// REMEMBER TO IntMasterEnable() IN MAIN AFTER INITS
void uart_interrupt_init()
{
    IntRegister(INT_UART1, uart_interrupt_handler); // Register handler
    UART1_ICR_R = 0xFFFFFFFF;   // Clear flags
    UART1_IM_R |= 0x10; // Interrupt on receive
    NVIC_EN0_R |= 0x40; // Enable UART1 in NVIC
}

void uart_interrupt_handler()
{
    if ((UART1_MIS_R & 0x10) != 0) // Check MIS
    {

        uart_data = uart_receive(); // Copy data
        flag = 1;

        UART1_ICR_R = 0x10; // Clear interrupts

    }
}
