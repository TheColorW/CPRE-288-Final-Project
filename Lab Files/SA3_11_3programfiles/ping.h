/*
 * ping.h
 *
 *  Created on: Oct 28, 2026
 *      Author: carlos28
 */

#ifndef PING_H_
#define PING_H_

void ping_init(void);

int ping_read(void);

void ping_handler(void);

extern volatile int overflow_count;

#endif /* PING_H_ */
