/*
 *   movement.h
 *
 *   Created on: Aug 25, 2026
 *   Updated on: Nov 18, 2026
 *      Authors: carlos28, jfr72506, gm3vf
 */

#include "open_interface.h"

// Move forward at a specified speed
double move_forward(oi_t *sensor, double centimeters, int speed);

// Move backward at a specified speed
void move_backward(oi_t *sensor, double  centimeters, int speed);

// Turn clockwise
void turn_clockwise(oi_t *sensor, int degrees);

// Turn anticlockwise
void turn_anticlockwise(oi_t *sensor, int degrees);

// Turn to a specified angle
void turn_to_angle(oi_t *sensor, int degrees);

// TODO
void go_around(oi_t *sensor, int direction);

extern volatile int flag;


double move_forward_until_stop(oi_t *sensor, double centimeters, int speed);

