/*
 * ping.h
 *
 *  Created on: Oct 28, 2026
 *      Author: carlos28
 */

#ifndef PING_H_
#define PING_H_

void ping_init(void);

// Returns raw 16 MHz timer ticks of echo pulse, or -1 on timeout
int ping_read(void);

// Returns distance in centimeters, or -1.0f on timeout
float ping_read_cm(void);

void TIMER3B_handler(void);

extern volatile int overflow_count;

#endif /* PING_H_ */
