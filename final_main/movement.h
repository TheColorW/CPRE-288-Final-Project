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

// Per-sensor cliff thresholds calibrated directly from lab floor measurements:
//   Black hole        : L:33   FL:16   FR:62   R:8
//   Normal gray tile  : L:1946 FL:1522 FR:1989 R:1264
//   White wall tape   : L:2776 FL:2387 FR:2612 R:2689
//   Final destination : L:3852 FL:2864 FR:3174 R:2923
#define CLIFF_HOLE_THRESH       150

#define CLIFF_WALL_L_THRESH     2350
#define CLIFF_WALL_FL_THRESH    2150
#define CLIFF_WALL_FR_THRESH    2300
#define CLIFF_WALL_R_THRESH     2250

// Destination mat detection (ENABLED - checked BEFORE white wall since values are higher!)
#define ENABLE_DEST_DETECT      1
#define CLIFF_DEST_L_THRESH     3150
#define CLIFF_DEST_FL_THRESH    2650
#define CLIFF_DEST_FR_THRESH    2880
#define CLIFF_DEST_R_THRESH     2810

// Default calibration constants (also live-tunable over UART with 'k' commands!)
// Calibrated from lab runs:
//   9.85 -> 109.5 cm  => 9.85 * (100 / 109.5) = 9.00 mm/cm
//   motor L=1.00, R=0.99 => dead straight
//   raw encoder angle scale ~1.00 with ~3.2 deg stopping coast lead
//   IMUPLUS (0x08) verified accurate within +/-1 deg -> 60% sanity blend
#define DEFAULT_MM_PER_CM_CAL     9.00f
#define DEFAULT_ANGLE_SCALE_CW    1.00f
#define DEFAULT_ANGLE_SCALE_CCW   1.00f
#define TURN_COAST_LEAD_DEG       3.2f
#define DEFAULT_MOTOR_CAL_L       1.00f
#define DEFAULT_MOTOR_CAL_R       0.99f
#define DEFAULT_IMU_BLEND         0.60f  // 0.0 = pure encoders, 0.6 = 60% IMUPLUS blend
#define IMU_MAX_AGREE_DIFF_DEG    15.0f  // Ignore IMU on a turn if it disagrees by >15 deg

// Live calibration state (modifiable via UART without reflashing)
extern float cal_mm_per_cm;
extern float cal_angle_scale_cw;
extern float cal_angle_scale_ccw;
extern float cal_imu_blend;
extern int   imu_enabled;

// Odometry callback called each oi_update() tick during tracked moves/turns
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

double move_forward_until_stop(oi_t *sensor, double centimeters, int speed);

// Encoder-primary turns & drives with continuous odometry + IMUPLUS redundancy
float  turn_by_degrees(oi_t *sensor, float delta_cw_deg,
                       float *out_enc_deg, float *out_imu_deg);

double move_forward_tracked(oi_t *sensor, double centimeters, int speed,
                            odom_step_cb_t on_step, float *out_imu_drift_deg);

double move_backward_tracked(oi_t *sensor, double centimeters, int speed,
                             odom_step_cb_t on_step);

#endif /* MOVEMENT_H_ */
