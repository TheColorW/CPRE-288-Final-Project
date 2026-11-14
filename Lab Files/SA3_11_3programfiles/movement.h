#include "open_interface.h"

double move_forward(oi_t * sensor, double centimeters);
void move_backward(oi_t *sensor, double centimeters);
void turn_clockwise(oi_t *sensor, int degrees);
void turn_counter_clockwise(oi_t *sensor, int degrees);
void go_around(oi_t *sensor, int direction);
void uart_sendString(char *string);

