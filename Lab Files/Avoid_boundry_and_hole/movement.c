/*
 *   movement.c
 *
 *   Created on: Aug 25, 2026
 *   Updated on: Nov 22, 2026
 *      Authors: carlos28, jfr72506, gm3vf, jjaritz
 *
 *   Wheel encoders are the primary driver for distance and turns.
 *   BNO055 (in gyro-only IMUPLUS mode 0x08) acts as a non-blocking
 *   redundancy / sanity check and stall detector.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "BNO055.h"
#include "movement.h"

volatile int flag = MOVE_FLAG_NONE;

// Live-tunable calibration parameters
float cal_mm_per_cm       = DEFAULT_MM_PER_CM_CAL;
float cal_angle_scale_cw  = DEFAULT_ANGLE_SCALE_CW;
float cal_angle_scale_ccw = DEFAULT_ANGLE_SCALE_CCW;
float cal_imu_blend       = DEFAULT_IMU_BLEND;
int   imu_enabled         = 1;

float normalize_deg(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f)    deg += 360.0f;
    return deg;
}

float angle_diff_deg(float target_deg, float current_deg)
{
    float err = target_deg - current_deg;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

int imu_read_heading_safe(float *out_deg)
{
    if (!imu_enabled || out_deg == 0)
    {
        return 0;
    }
    float h = 0.0f;
    if (bno055_convert_float_euler_h_deg(&h) != 0)
    {
        return 0;
    }
    // Reject NaN or out-of-range garbage values
    if (h != h || h < -360.0f || h > 720.0f)
    {
        return 0;
    }
    *out_deg = normalize_deg(h);
    return 1;
}

static int check_hazards(oi_t *sensor)
{
    int leftCliff       = sensor->cliffLeftSignal;
    int frontLeftCliff  = sensor->cliffFrontLeftSignal;
    int rightCliff      = sensor->cliffRightSignal;
    int frontRightCliff = sensor->cliffFrontRightSignal;

    // 1. Physical bumpers
    if (sensor->bumpLeft && sensor->bumpRight)
    {
        return MOVE_FLAG_BUMP_BOTH;
    }
    if (sensor->bumpLeft)
    {
        return MOVE_FLAG_BUMP_LEFT;
    }
    if (sensor->bumpRight)
    {
        return MOVE_FLAG_BUMP_RIGHT;
    }

    // 2. White boundary tape (walls)
    if (frontLeftCliff > CLIFF_WALL_THRESH || frontRightCliff > CLIFF_WALL_THRESH)
    {
        return MOVE_FLAG_WALL_FRONT;
    }
    if (leftCliff > CLIFF_WALL_THRESH)
    {
        return MOVE_FLAG_WALL_LEFT;
    }
    if (rightCliff > CLIFF_WALL_THRESH)
    {
        return MOVE_FLAG_WALL_RIGHT;
    }

    // 3. Black holes (floor drops)
    if (frontLeftCliff < CLIFF_HOLE_THRESH || frontRightCliff < CLIFF_HOLE_THRESH)
    {
        return MOVE_FLAG_HOLE_FRONT;
    }
    if (leftCliff < CLIFF_HOLE_THRESH)
    {
        return MOVE_FLAG_HOLE_LEFT;
    }
    if (rightCliff < CLIFF_HOLE_THRESH)
    {
        return MOVE_FLAG_HOLE_RIGHT;
    }

#if ENABLE_DEST_DETECT
    // 4. Destination paper on floor
    if ((frontLeftCliff  >= CLIFF_DEST_MIN && frontLeftCliff  <= CLIFF_DEST_MAX) ||
        (frontRightCliff >= CLIFF_DEST_MIN && frontRightCliff <= CLIFF_DEST_MAX) ||
        (leftCliff       >= CLIFF_DEST_MIN && leftCliff       <= CLIFF_DEST_MAX) ||
        (rightCliff      >= CLIFF_DEST_MIN && rightCliff      <= CLIFF_DEST_MAX))
    {
        return MOVE_FLAG_DESTINATION;
    }
#endif

    return MOVE_FLAG_NONE;
}

// =========================================================================
// Basic Movement Helpers (Encoder-Primary)
// =========================================================================

double move_forward(oi_t *sensor, double centimeters, int speed)
{
    double sum_mm = 0.0;
    double target_mm = centimeters * (double)cal_mm_per_cm;

    oi_update(sensor);
    oi_setWheels(speed, speed);

    while (sum_mm < target_mm)
    {
        oi_update(sensor);
        sum_mm += sensor->distance;
        if (sensor->bumpLeft || sensor->bumpRight)
        {
            break;
        }
    }

    oi_setWheels(0, 0);
    return sum_mm / (double)cal_mm_per_cm;
}

double move_backward(oi_t *sensor, double centimeters, int speed)
{
    int back_speed = -abs(speed);
    double sum_mm = 0.0;
    double target_mm = centimeters * (double)cal_mm_per_cm;

    oi_update(sensor);
    oi_setWheels(back_speed, back_speed);

    while (fabs(sum_mm) < target_mm)
    {
        oi_update(sensor);
        sum_mm += fabs(sensor->distance);
    }

    oi_setWheels(0, 0);
    return sum_mm / (double)cal_mm_per_cm;
}

void turn_clockwise(oi_t *sensor, int degrees)
{
    turn_by_degrees(sensor, (float)abs(degrees), 0, 0);
}

void turn_anticlockwise(oi_t *sensor, int degrees)
{
    turn_by_degrees(sensor, -(float)abs(degrees), 0, 0);
}

void turn_to_angle(oi_t *sensor, int degrees)
{
    if (degrees < 90)
    {
        turn_by_degrees(sensor, (float)(90 - degrees), 0, 0);
    }
    else if (degrees > 90)
    {
        turn_by_degrees(sensor, -(float)(degrees - 90), 0, 0);
    }
}

void go_around(oi_t *sensor_data, int direction)
{
    timer_waitMillis(200);
    move_backward(sensor_data, 15, 140);
    timer_waitMillis(200);

    if (direction == 0)
    {
        turn_clockwise(sensor_data, 90);
        timer_waitMillis(200);
        move_forward(sensor_data, 35, 140);
        timer_waitMillis(200);
        turn_anticlockwise(sensor_data, 90);
    }
    else
    {
        turn_anticlockwise(sensor_data, 90);
        timer_waitMillis(200);
        move_forward(sensor_data, 35, 140);
        timer_waitMillis(200);
        turn_clockwise(sensor_data, 90);
    }
    timer_waitMillis(200);
}

double move_forward_until_stop(oi_t *sensor, double centimeters, int speed)
{
    return move_forward_tracked(sensor, centimeters, speed, 0, 0);
}

// =========================================================================
// Encoder-Primary Turn with Two-Stage Speed + IMU Redundancy
// =========================================================================

float turn_by_degrees(oi_t *sensor, float delta_cw_deg,
                      float *out_enc_deg, float *out_imu_deg)
{
    // Wrap requested delta to shortest turn in [-180, +180]
    while (delta_cw_deg > 180.0f)  delta_cw_deg -= 360.0f;
    while (delta_cw_deg < -180.0f) delta_cw_deg += 360.0f;

    float target_abs = fabsf(delta_cw_deg);
    if (target_abs < 1.5f)
    {
        if (out_enc_deg) *out_enc_deg = 0.0f;
        if (out_imu_deg) *out_imu_deg = 0.0f;
        return 0.0f;
    }

    int is_cw = (delta_cw_deg > 0.0f) ? 1 : 0;
    float scale = is_cw ? cal_angle_scale_cw : cal_angle_scale_ccw;

    float imu_start = 0.0f;
    int imu_ok_start = imu_read_heading_safe(&imu_start);

    oi_update(sensor); // Flush stale encoder angle

    float turned_cw_deg = 0.0f;
    int stalled_ticks = 0;
    int ticks = 0;

    while (fabsf(turned_cw_deg) < target_abs && ticks < 350)
    {
        float remaining = target_abs - fabsf(turned_cw_deg);
        // Two-stage speed minimizes stopping overshoot
        int16_t spd = (remaining > 18.0f) ? 100 : 40;

        if (is_cw)
        {
            // CW: right wheel back, left wheel forward
            oi_setWheels(-spd, spd);
        }
        else
        {
            // CCW: right wheel forward, left wheel back
            oi_setWheels(spd, -spd);
        }

        oi_update(sensor);
        ticks++;

        // open_interface.c: sensor->angle is positive CCW, negative CW
        float step_cw = (float)(-sensor->angle) * scale;
        turned_cw_deg += step_cw;

        if (fabsf(step_cw) < 0.05f)
        {
            stalled_ticks++;
            if (stalled_ticks > 40) // ~1 second with zero wheel turn -> stuck
            {
                uart_sendStr("WARN_TURN_STALL\r\n");
                break;
            }
        }
        else
        {
            stalled_ticks = 0;
        }
    }

    oi_setWheels(0, 0);
    timer_waitMillis(120); // Let chassis settle

    // Capture any tiny coasting after wheels stop
    oi_update(sensor);
    turned_cw_deg += (float)(-sensor->angle) * scale;

    // IMU redundancy cross-check (never overrides if IMU jumped!)
    float imu_end = 0.0f;
    float imu_delta_cw = turned_cw_deg;
    float final_cw_deg = turned_cw_deg;

    if (imu_ok_start && imu_read_heading_safe(&imu_end))
    {
        imu_delta_cw = angle_diff_deg(imu_end, imu_start);
        float diff = fabsf(imu_delta_cw - turned_cw_deg);

        if (diff <= IMU_MAX_AGREE_DIFF_DEG && cal_imu_blend > 0.0f)
        {
            // IMU agrees reasonably with encoders -> gently blend as redundancy
            final_cw_deg = turned_cw_deg + cal_imu_blend * (imu_delta_cw - turned_cw_deg);
        }
        else if (diff > IMU_MAX_AGREE_DIFF_DEG)
        {
            char buf[72];
            sprintf(buf, "IMU_IGNORED enc:%.1f imu:%.1f\r\n", turned_cw_deg, imu_delta_cw);
            uart_sendStr(buf);
        }
    }

    if (out_enc_deg) *out_enc_deg = turned_cw_deg;
    if (out_imu_deg) *out_imu_deg = imu_delta_cw;
    return final_cw_deg;
}

// =========================================================================
// Encoder-Primary Straight Drive with Per-Tick Odometry + IMU Drift Check
// =========================================================================

double move_forward_tracked(oi_t *sensor, double centimeters, int speed,
                            odom_step_cb_t on_step, float *out_imu_drift_deg)
{
    double sum_mm = 0.0;
    double target_mm = centimeters * (double)cal_mm_per_cm;
    float imu_start = 0.0f;
    int imu_ok_start = imu_read_heading_safe(&imu_start);

    flag = MOVE_FLAG_NONE;
    oi_update(sensor); // Flush stale deltas

    // Ramp down near end for accurate stopping distance
    oi_setWheels(speed, speed);

    while (sum_mm < target_mm)
    {
        double rem_cm = (target_mm - sum_mm) / (double)cal_mm_per_cm;
        int cur_speed = (rem_cm < 8.0 && speed > 70) ? 70 : speed;
        oi_setWheels(cur_speed, cur_speed);

        oi_update(sensor);
        double step_mm = sensor->distance;
        sum_mm += step_mm;

        // Track tiny wheel-encoder heading drift during straight segments
        float raw_cw = (float)(-sensor->angle);
        float scale = (raw_cw >= 0.0f) ? cal_angle_scale_cw : cal_angle_scale_ccw;
        float step_cw_deg = raw_cw * scale;

        if (on_step != 0 && (step_mm != 0.0 || step_cw_deg != 0.0f))
        {
            on_step((float)(step_mm / (double)cal_mm_per_cm), step_cw_deg);
        }

        int hazard = check_hazards(sensor);
        if (hazard != MOVE_FLAG_NONE)
        {
            flag = hazard;
            break;
        }
    }

    oi_setWheels(0, 0);
    timer_waitMillis(80);

    // Capture small coasting distance after stopping
    oi_update(sensor);
    if (sensor->distance != 0.0)
    {
        sum_mm += sensor->distance;
        if (on_step != 0)
        {
            float raw_cw = (float)(-sensor->angle);
            float scale = (raw_cw >= 0.0f) ? cal_angle_scale_cw : cal_angle_scale_ccw;
            on_step((float)(sensor->distance / (double)cal_mm_per_cm), raw_cw * scale);
        }
    }

    float imu_end = 0.0f;
    float drift = 0.0f;
    if (imu_ok_start && imu_read_heading_safe(&imu_end))
    {
        drift = angle_diff_deg(imu_end, imu_start);
    }
    if (out_imu_drift_deg) *out_imu_drift_deg = drift;

    return sum_mm / (double)cal_mm_per_cm;
}

double move_backward_tracked(oi_t *sensor, double centimeters, int speed,
                             odom_step_cb_t on_step)
{
    int back_speed = -abs(speed);
    double sum_mm = 0.0;
    double target_mm = centimeters * (double)cal_mm_per_cm;

    oi_update(sensor);
    oi_setWheels(back_speed, back_speed);

    while (fabs(sum_mm) < target_mm)
    {
        oi_update(sensor);
        double step_mm = sensor->distance; // Negative when reversing
        sum_mm += fabs(step_mm);

        float raw_cw = (float)(-sensor->angle);
        float scale = (raw_cw >= 0.0f) ? cal_angle_scale_cw : cal_angle_scale_ccw;
        float step_cw_deg = raw_cw * scale;

        if (on_step != 0 && (step_mm != 0.0 || step_cw_deg != 0.0f))
        {
            on_step(-(float)(fabs(step_mm) / (double)cal_mm_per_cm), step_cw_deg);
        }
    }

    oi_setWheels(0, 0);
    timer_waitMillis(80);
    oi_update(sensor);
    return sum_mm / (double)cal_mm_per_cm;
}
