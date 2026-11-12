#ifndef PING_H_
#define PING_H_

// Set up PB3 and Timer 3B input capture.
void ping_init(void);

// Trigger one ping and return its echo width in clock cycles, or -1 on timeout.
int ping_read(void);

// Return the running count of rollovers that occurred during measured echoes.
unsigned int ping_get_wrap_count(void);

#endif
