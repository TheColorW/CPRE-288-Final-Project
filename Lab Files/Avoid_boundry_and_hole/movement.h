/*
 *   movement.h
 *
 *   Created on: Aug 25, 2026
 *   Updated on: Nov 22, 2026
 *      Authors: carlos28, jfr72506, gm3vf, jjaritz
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"

// Stop reason flags set by move_forward_until_stop / move_forward_imu
#define MOVE_FLAG_NONE        0
#define MOVE_FLAG_BUMP_LEFT   1
#define MOVE_FLAG_WALL_LEFT   2
#define MOVE_FLAG_WALL_RIGHT  3
#define MOVE_FLAG_WALL_FRONT  4
#define MOVE_FLAG_HOLE_LEFT   5
#define MOVE_FLAG_HOLE_RIGHT  6
#define MOVE_FLAG_HOLE_FRONT  7
#define MOVE_FLAG_DESTINATION 8
#define MOVE_FLAG_BUMP_RIGHT  9
#define MOVE_FLAG_BUMP_BOTH   10

// Cliff sensor thresholds (calibrated from lab floor measurements)
// Black hole: ~50, Normal gray floor: ~2500, White wall tape: ~2800
#define CLIFF_WALL_THRESH     2650
#define CLIFF_HOLE_THRESH     120

// Destination mat detection (paper on floor)
// TODO: Use 'p' UART command to read cliff values on destination paper,
//       update CLIFF_DEST_MIN / CLIFF_DEST_MAX, then set ENABLE_DEST_DETECT to 1.
#define ENABLE_DEST_DETECT    0
#define CLIFF_DEST_MIN        400
#define CLIFF_DEST_MAX        2100

// Distance calibration: raw oi_t->distance ticks (mm) per actual cm traveled.
// Original code used (cm * 10 - 1.5 * cm) = 8.5 mm/cm. Tune on a 100 cm test drive.
#define MM_PER_CM_CAL         8.5

// IMU turn control tuning
#define HEADING_TOL_DEG       1.5f
#define HEADING_KP            3.0f
#define HEADING_MAX_CORR      40

// Callback invoked each oi_update tick during IMU/odometry moves
typedef void (*move_step_cb_t)(float delta_cm, float imu_heading_deg);

extern volatile int flag;

// Basic movement functions (return actual cm traveled)
double move_forward(oi_t *sensor, double centimeters, int speed);
double move_backward(oi_t *sensor, double centimeters, int speed);
void turn_clockwise(oi_t *sensor, int degrees);
void turn_anticlockwise(oi_t *sensor, int degrees);
void turn_to_angle(oi_t *sensor, int degrees);
void go_around(oi_t *sensor, int direction);

// Moves forward until target distance or bumper/wall/hole/destination trigger
double move_forward_until_stop(oi_t *sensor, double centimeters, int speed);

// IMU-assisted closed-loop turn & straight drive with continuous odometry callback
float normalize_deg(float deg);
float angle_diff_deg(float target_deg, float current_deg);
void turn_to_heading(oi_t *sensor, float target_deg);
double move_forward_imu(oi_t *sensor, double centimeters, int speed,
                        float target_imu_deg, move_step_cb_t on_step);
double move_backward_odometry(oi_t *sensor, double centimeters, int speed,
                              move_step_cb_t on_step);

#endif /* MOVEMENT_H_ */
