/*
 *   movement.h
 *
 *   Created on: Aug 25, 2026
 *   Updated on: Nov 22, 2026
 *      Authors: carlos28, jfr72506, gm3vf, jjaritz
 *
 *   Encoder-primary movement & calibration with BNO055 IMUPLUS (0x08) redundancy.
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"

// Stop reason flags set by move_forward_until_stop / move_forward_tracked
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
// TODO: Use 'p' UART command on destination paper, update MIN/MAX, set ENABLE to 1.
#define ENABLE_DEST_DETECT    0
#define CLIFF_DEST_MIN        400
#define CLIFF_DEST_MAX        2100

// Default calibration constants (also live-tunable over UART with 'k' commands!)
// Run 't d 100' and 't a 90' / 't a -90' before test runs to verify accuracy.
#define DEFAULT_MM_PER_CM_CAL     10.0f
#define DEFAULT_ANGLE_SCALE_CW    1.00f
#define DEFAULT_ANGLE_SCALE_CCW   1.00f
#define DEFAULT_IMU_BLEND         0.20f  // 0.0 = pure encoders, 0.2 = 20% IMU sanity blend
#define IMU_MAX_AGREE_DIFF_DEG    15.0f  // Ignore IMU on a turn if it disagrees by >15 deg

// Live calibration state (modifiable via UART without reflashing)
extern float cal_mm_per_cm;
extern float cal_angle_scale_cw;
extern float cal_angle_scale_ccw;
extern float cal_imu_blend;
extern int   imu_enabled;

// Odometry callback called each oi_update() tick during tracked moves/turns
// delta_cm: forward (+), backward (-)
// delta_cw_deg: clockwise (+), counterclockwise (-)
typedef void (*odom_step_cb_t)(float delta_cm, float delta_cw_deg);

extern volatile int flag;

// Angle utilities
float normalize_deg(float deg);
float angle_diff_deg(float target_deg, float current_deg);
int   imu_read_heading_safe(float *out_deg);

// Basic movement functions (return actual calibrated cm traveled)
double move_forward(oi_t *sensor, double centimeters, int speed);
double move_backward(oi_t *sensor, double centimeters, int speed);
void   turn_clockwise(oi_t *sensor, int degrees);
void   turn_anticlockwise(oi_t *sensor, int degrees);
void   turn_to_angle(oi_t *sensor, int degrees);
void   go_around(oi_t *sensor, int direction);

// Moves forward until target distance or bump/wall/hole/destination trigger
double move_forward_until_stop(oi_t *sensor, double centimeters, int speed);

// Encoder-primary turns & drives with continuous odometry + IMU redundancy
// turn_by_degrees: delta_cw_deg > 0 turns CW (right), < 0 turns CCW (left).
// Returns estimated actual CW degrees turned (encoder primary + IMU redundancy).
float  turn_by_degrees(oi_t *sensor, float delta_cw_deg,
                       float *out_enc_deg, float *out_imu_deg);

double move_forward_tracked(oi_t *sensor, double centimeters, int speed,
                            odom_step_cb_t on_step, float *out_imu_drift_deg);

double move_backward_tracked(oi_t *sensor, double centimeters, int speed,
                             odom_step_cb_t on_step);

#endif /* MOVEMENT_H_ */
