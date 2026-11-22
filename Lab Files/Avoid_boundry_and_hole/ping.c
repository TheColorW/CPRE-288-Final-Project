/*
 * ping.c
 *
 * Upgraded with timeout-safe Timer3B input-capture ISR (from SA2/PING).
 */

#include <stdbool.h>
#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "Timer.h"
#include "driverlib/interrupt.h"
#include "ping.h"

#define PORTB_CLOCK_MASK     0x02U
#define TIMER3_CLOCK_MASK    0x08U
#define PING_MASK            0x08U
#define PB3_PCTL_MASK        (0xFU << 12)  // PB3 uses PCTL bits 15:12
#define PB3_T3CCP1           (0x7U << 12)  // Function 7 routes PB3 to Timer 3B
#define TIMER3B_ENABLE       (1U << 8)
#define TIMER3B_BOTH_EDGES   (0x3U << 10)  // TBEVENT field in GPTMCTL
#define CAPTURE_EVENT        (1U << 10)    // Capture B event in IMR, MIS, ICR
#define TIMER_COUNT_MASK     0x00FFFFFFU
#define TIMER_PERIOD_COUNTS  (TIMER_COUNT_MASK + 1U)
#define ECHO_TIMEOUT_US      30000U        // 30 ms (~5 m max range)

volatile int overflow_count = 0;

static volatile bool waiting_for_falling = false;
static volatile int echo_cycles = -1;
static uint32_t rising_count = 0;

void TIMER3B_handler(void)
{
    if ((TIMER3_MIS_R & CAPTURE_EVENT) == 0U)
    {
        return;
    }

    uint32_t captured = TIMER3_TBR_R & TIMER_COUNT_MASK;
    TIMER3_ICR_R = CAPTURE_EVENT;

    if (!waiting_for_falling)
    {
        rising_count = captured;
        waiting_for_falling = true;
    }
    else
    {
        // Mask capture interrupts until next ping_read()
        TIMER3_IMR_R &= ~CAPTURE_EVENT;

        int cycles = (int)rising_count - (int)captured;
        if (cycles < 0)
        {
            cycles += (int)TIMER_PERIOD_COUNTS;
            overflow_count++;
        }

        echo_cycles = cycles;
    }
}

void ping_init(void)
{
    SYSCTL_RCGCGPIO_R |= PORTB_CLOCK_MASK;
    SYSCTL_RCGCTIMER_R |= TIMER3_CLOCK_MASK;
    while ((SYSCTL_PRGPIO_R & PORTB_CLOCK_MASK) == 0U ||
           (SYSCTL_PRTIMER_R & TIMER3_CLOCK_MASK) == 0U)
    {
        // Wait for Port B and Timer 3 clocks to stabilize
    }

    GPIO_PORTB_DIR_R &= ~PING_MASK;
    GPIO_PORTB_AFSEL_R &= ~PING_MASK;
    GPIO_PORTB_AMSEL_R &= ~PING_MASK;
    GPIO_PORTB_ODR_R &= ~PING_MASK;
    GPIO_PORTB_DEN_R |= PING_MASK;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~PB3_PCTL_MASK) | PB3_T3CCP1;

    TIMER3_CTL_R &= ~TIMER3B_ENABLE;
    TIMER3_CFG_R = 0x04U;
    TIMER3_TBMR_R = 0x07U;          // Capture mode, edge-time, count down
    TIMER3_TBILR_R = 0xFFFFU;
    TIMER3_TBPR_R = 0xFFU;
    TIMER3_CTL_R |= TIMER3B_BOTH_EDGES;

    TIMER3_IMR_R &= ~CAPTURE_EVENT;
    TIMER3_ICR_R = CAPTURE_EVENT;
    IntRegister(INT_TIMER3B, TIMER3B_handler);
    IntEnable(INT_TIMER3B);
    TIMER3_CTL_R |= TIMER3B_ENABLE;
}

static void ping_trigger(void)
{
    GPIO_PORTB_AFSEL_R &= ~PING_MASK;
    GPIO_PORTB_DIR_R |= PING_MASK;
    GPIO_PORTB_DATA_R &= ~PING_MASK;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R |= PING_MASK;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R &= ~PING_MASK;

    GPIO_PORTB_DIR_R &= ~PING_MASK;
    GPIO_PORTB_AFSEL_R |= PING_MASK;
}

int ping_read(void)
{
    TIMER3_IMR_R &= ~CAPTURE_EVENT;
    waiting_for_falling = false;
    echo_cycles = -1;

    ping_trigger();

    // Discard any spurious edge from switching pin direction
    TIMER3_ICR_R = CAPTURE_EVENT;
    TIMER3_IMR_R |= CAPTURE_EVENT;

    uint32_t start_us = timer_getMicros();
    while (echo_cycles < 0 && (timer_getMicros() - start_us) < ECHO_TIMEOUT_US)
    {
        // Wait for echo or timeout
    }

    TIMER3_IMR_R &= ~CAPTURE_EVENT;
    return echo_cycles;
}

float ping_read_cm(void)
{
    int ticks = ping_read();
    if (ticks <= 0)
    {
        return -1.0f;
    }
    double pulse_ms = (double)ticks / 16000.0;
    double dist_cm = (pulse_ms * 34.3) / 2.0 - 4.0;
    if (dist_cm < 0.0)
    {
        dist_cm = 0.0;
    }
    return (float)dist_cm;
}
