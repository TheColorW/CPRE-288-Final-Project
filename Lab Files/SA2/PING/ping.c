#include <stdbool.h>
#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "Timer.h"
#include "driverlib/interrupt.h"
#include "ping.h"

#define PORTB_CLOCK_MASK     0x02
#define TIMER3_CLOCK_MASK    0x08
#define PING_MASK            0x08
#define PB3_PCTL_MASK        (0xF << 12)  // PB3 uses PCTL bits 15:12.
#define PB3_T3CCP1           (0x7 << 12)  // Function 7 routes PB3 to Timer 3B.
#define TIMER3B_ENABLE       (1 << 8)
#define TIMER3B_BOTH_EDGES   (0x3 << 10)  // TBEVENT field in GPTMCTL.
#define CAPTURE_EVENT        (1 << 10)    // Capture B event in IMR, MIS and ICR.
#define TIMER_COUNT_MASK     0x00FFFFFF
#define TIMER_PERIOD_COUNTS (TIMER_COUNT_MASK + 1)  // 2^24 timer counts.
#define ECHO_TIMEOUT_US      30000

// These variables are shared between the ISR and the main program.
static volatile bool waiting_for_falling;
static volatile int echo_cycles;
static volatile unsigned int wrap_count;

// Only the capture ISR reads and writes this timestamp.
static uint32_t rising_count;

static void ping_capture_isr(void)
{
    // Check that this interrupt came from an enabled Timer 3B capture event.
    if ((TIMER3_MIS_R & CAPTURE_EVENT) == 0)
    {
        return;
    }

    // TBR holds the count at the edge, including the upper 8 prescale bits.
    uint32_t captured = TIMER3_TBR_R & TIMER_COUNT_MASK;
    TIMER3_ICR_R = CAPTURE_EVENT;  // Write 1 to acknowledge this interrupt.

    if (!waiting_for_falling)
    {
        rising_count = captured;
        waiting_for_falling = true;
    }
    else
    {
        // Stop accepting edges once the pair is complete.
        TIMER3_IMR_R &= ~CAPTURE_EVENT;

        int cycles = (int)rising_count - (int)captured;
        if (cycles < 0)
        {
            // The countdown passed zero and reloaded between these edges.
            // An echo is shorter than one full timer period.
            cycles += (int)TIMER_PERIOD_COUNTS;
            ++wrap_count;
        }

        echo_cycles = cycles;
    }
}

void ping_init(void)
{
    SYSCTL_RCGCGPIO_R |= PORTB_CLOCK_MASK;
    SYSCTL_RCGCTIMER_R |= TIMER3_CLOCK_MASK;
    while ((SYSCTL_PRGPIO_R & PORTB_CLOCK_MASK) == 0U ||
           (SYSCTL_PRTIMER_R & TIMER3_CLOCK_MASK) == 0)
    {
        // Wait for Port B and Timer 3 to be ready before using their registers.
    }

    GPIO_PORTB_DIR_R &= ~PING_MASK;
    GPIO_PORTB_AFSEL_R &= ~PING_MASK;
    GPIO_PORTB_AMSEL_R &= ~PING_MASK;
    GPIO_PORTB_ODR_R &= ~PING_MASK;
    GPIO_PORTB_DEN_R |= PING_MASK;
    // Set the alternate function now. AFSEL connects it after each trigger.
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~PB3_PCTL_MASK) | PB3_T3CCP1;

    TIMER3_CTL_R &= ~TIMER3B_ENABLE;
    TIMER3_CFG_R = 0x04U;           // Split Timer 3 into two 16-bit halves.
    TIMER3_TBMR_R = 0x07U;          // Capture mode, edge-time, counting down.
    TIMER3_TBILR_R = 0xFFFFU;       // Load the low 16 bits of the countdown.
    TIMER3_TBPR_R = 0xFFU;          // Extend the countdown by 8 bits.
    TIMER3_CTL_R |= TIMER3B_BOTH_EDGES;

    // Leave capture interrupts masked until PB3 is listening for an echo.
    TIMER3_IMR_R &= ~CAPTURE_EVENT;
    TIMER3_ICR_R = CAPTURE_EVENT;
    IntRegister(INT_TIMER3B, ping_capture_isr);
    IntEnable(INT_TIMER3B);
    TIMER3_CTL_R |= TIMER3B_ENABLE;
}

static void ping_trigger(void)
{
    // Temporarily take PB3 away from the timer and drive the trigger as GPIO.
    GPIO_PORTB_AFSEL_R &= ~PING_MASK;
    GPIO_PORTB_DIR_R |= PING_MASK;
    GPIO_PORTB_DATA_R &= ~PING_MASK;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R |= PING_MASK;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R &= ~PING_MASK;

    // Release the shared wire, then connect the input to T3CCP1.
    GPIO_PORTB_DIR_R &= ~PING_MASK;
    GPIO_PORTB_AFSEL_R |= PING_MASK;
}

int ping_read(void)
{
    TIMER3_IMR_R &= ~CAPTURE_EVENT;
    waiting_for_falling = false;
    echo_cycles = -1;

    ping_trigger();

    // Discard any event from switching PB3 before arming for the echo.
    TIMER3_ICR_R = CAPTURE_EVENT;
    TIMER3_IMR_R |= CAPTURE_EVENT;

    uint32_t start_us = timer_getMicros();
    while (echo_cycles < 0 && (timer_getMicros() - start_us) < ECHO_TIMEOUT_US)
    {
        // Wait for the falling-edge ISR or time out after 30 ms.
    }

    TIMER3_IMR_R &= ~CAPTURE_EVENT;
    return echo_cycles;
}

unsigned int ping_get_wrap_count(void)
{
    return wrap_count;
}
