/*
 *   movement.c
 *
 *   Created on: Aug 25, 2026
 *   Updated on: Nov 22, 2026
 *      Authors: carlos28, jfr72506, gm3vf, jjaritz
 */

#include <math.h>
#include <stdlib.h>
#include "Timer.h"
#include "lcd.h"
#include "BNO055.h"
#include "movement.h"

volatile int flag = MOVE_FLAG_NONE;

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

// Move forward at a specified speed, returns actual cm traveled
double move_forward(oi_t *sensor, double centimeters, int speed)
{
    double sum = 0.0;
    double target_mm = centimeters * MM_PER_CM_CAL;

    oi_update(sensor); // Flush stale distance delta
    oi_setWheels(speed, speed);

    while (sum < target_mm)
    {
        oi_update(sensor);
        sum += sensor->distance;

        if (sensor->bumpLeft || sensor->bumpRight)
        {
            break;
        }
    }

    oi_setWheels(0, 0);
    return sum / MM_PER_CM_CAL;
}

// Move backward at a specified speed (positive speed argument drives backward)
double move_backward(oi_t *sensor, double centimeters, int speed)
{
    int back_speed = -abs(speed);
    double sum = 0.0;
    double target_mm = centimeters * MM_PER_CM_CAL;

    oi_update(sensor); // Flush stale distance delta
    oi_setWheels(back_speed, back_speed);

    while (fabs(sum) < target_mm)
    {
        oi_update(sensor);
        sum += fabs(sensor->distance);
    }

    oi_setWheels(0, 0);
    return sum / MM_PER_CM_CAL;
}

// Turn clockwise using Roomba wheel encoders
void turn_clockwise(oi_t *sensor, int degrees)
{
    double sum = 0.0;
    oi_update(sensor);
    oi_setWheels(-150, 150);

    while (fabs(sum) < (degrees - 2))
    {
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0, 0);
}

// Turn anticlockwise using Roomba wheel encoders
void turn_anticlockwise(oi_t *sensor, int degrees)
{
    double sum = 0.0;
    oi_update(sensor);
    oi_setWheels(150, -150);

    while (fabs(sum) < (degrees - 2))
    {
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0, 0);
}

void turn_to_angle(oi_t *sensor, int degrees)
{
    if (degrees < 90)
    {
        turn_clockwise(sensor, 90 - degrees);
    }
    else if (degrees > 90)
    {
        turn_anticlockwise(sensor, degrees - 90);
    }
}

void go_around(oi_t *sensor_data, int direction) // 0: right (left bump), 1: left (right bump)
{
    timer_waitMillis(250);
    move_backward(sensor_data, 15, 150);
    timer_waitMillis(250);

    if (direction == 0)
    {
        turn_clockwise(sensor_data, 90);
        timer_waitMillis(250);
        move_forward(sensor_data, 20, 150);
        timer_waitMillis(250);
        turn_anticlockwise(sensor_data, 90);
    }
    else
    {
        turn_anticlockwise(sensor_data, 90);
        timer_waitMillis(250);
        move_forward(sensor_data, 20, 150);
        timer_waitMillis(250);
        turn_clockwise(sensor_data, 90);
    }
    timer_waitMillis(250);
}

double move_forward_until_stop(oi_t *sensor, double centimeters, int speed)
{
    double sum = 0.0;
    double target_mm = centimeters * MM_PER_CM_CAL;

    flag = MOVE_FLAG_NONE;
    oi_update(sensor); // Flush stale distance delta
    oi_setWheels(speed, speed);

    while (sum < target_mm)
    {
        oi_update(sensor);
        sum += sensor->distance;

        int hazard = check_hazards(sensor);
        if (hazard != MOVE_FLAG_NONE)
        {
            flag = hazard;
            break;
        }
    }

    oi_setWheels(0, 0);
    return sum / MM_PER_CM_CAL;
}

void turn_to_heading(oi_t *sensor, float target_deg)
{
    float heading = 0.0f;
    float error = 0.0f;
    int16_t speed = 0;
    int iter = 0;

    target_deg = normalize_deg(target_deg);

    while (iter < 400) // ~6 second safety timeout
    {
        bno055_convert_float_euler_h_deg(&heading);
        error = angle_diff_deg(target_deg, heading);

        lcd_printf("Target: %.1f\nCurr:   %.1f\nErr:    %.1f",
                   target_deg, heading, error);

        if (fabsf(error) <= HEADING_TOL_DEG)
        {
            break;
        }

        // Two-stage speed control: fast far away, slow creep near target
        speed = (fabsf(error) > 15.0f) ? 80 : 30;

        if (error > 0.0f)
        {
            // Increase heading -> clockwise (right wheel back, left wheel forward)
            oi_setWheels(-speed, speed);
        }
        else
        {
            // Decrease heading -> counterclockwise
            oi_setWheels(speed, -speed);
        }

        timer_waitMillis(15);
        iter++;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(150);
    oi_update(sensor); // Flush spin encoder delta so next drive starts clean
}

double move_forward_imu(oi_t *sensor, double centimeters, int speed,
                        float target_imu_deg, move_step_cb_t on_step)
{
    double sum_mm = 0.0;
    double target_mm = centimeters * MM_PER_CM_CAL;
    float heading = 0.0f;

    target_imu_deg = normalize_deg(target_imu_deg);
    flag = MOVE_FLAG_NONE;

    oi_update(sensor); // Flush stale distance delta
    oi_setWheels(speed, speed);

    while (sum_mm < target_mm)
    {
        oi_update(sensor);
        double step_mm = sensor->distance;
        sum_mm += step_mm;

        bno055_convert_float_euler_h_deg(&heading);

        if (on_step != 0 && step_mm != 0.0)
        {
            on_step((float)(step_mm / MM_PER_CM_CAL), heading);
        }

        int hazard = check_hazards(sensor);
        if (hazard != MOVE_FLAG_NONE)
        {
            flag = hazard;
            break;
        }

        // Gentle proportional heading-hold so wheels don't veer off line
        float err = angle_diff_deg(target_imu_deg, heading);
        int corr = (int)(HEADING_KP * err);
        if (corr > HEADING_MAX_CORR)  corr = HEADING_MAX_CORR;
        if (corr < -HEADING_MAX_CORR) corr = -HEADING_MAX_CORR;

        // err > 0 => turn CW => right slower, left faster
        oi_setWheels((int16_t)(speed - corr), (int16_t)(speed + corr));
    }

    oi_setWheels(0, 0);
    return sum_mm / MM_PER_CM_CAL;
}

double move_backward_odometry(oi_t *sensor, double centimeters, int speed,
                              move_step_cb_t on_step)
{
    int back_speed = -abs(speed);
    double sum_mm = 0.0;
    double target_mm = centimeters * MM_PER_CM_CAL;
    float heading = 0.0f;

    oi_update(sensor); // Flush stale distance delta
    oi_setWheels(back_speed, back_speed);

    while (fabs(sum_mm) < target_mm)
    {
        oi_update(sensor);
        double step_mm = sensor->distance; // Negative when reversing
        sum_mm += fabs(step_mm);

        bno055_convert_float_euler_h_deg(&heading);

        if (on_step != 0 && step_mm != 0.0)
        {
            // Moving opposite to heading -> negative delta_cm
            on_step(-(float)(fabs(step_mm) / MM_PER_CM_CAL), heading);
        }
    }

    oi_setWheels(0, 0);
    return sum_mm / MM_PER_CM_CAL;
}
