/*
 * main.c
 *
 *  Created on: Oct 29, 2026
 *  Updated on: Nov 22, 2026
 *      Authors: carlos28, jjaritz
 *
 * Assumes robot starts in Bottom-Left (SW) corner (22.5, 22.5) facing NORTH (0 deg).
 * Wheel encoders are primary for driving/turning; BNO055 (IMUPLUS 0x08, gyro+accel)
 * is non-blocking redundancy only.
 *
 * UART Commands (via PuTTY / GUI):
 *   t d [cm]         : Distance calibration test (default 100 cm)
 *   t a [deg]        : Angle calibration test (+CW / -CCW, default 90 deg)
 *   t c              : Print current calibration scales + live sensor readings
 *   k d <mm_per_cm>  : Live-tune distance scale (default 10.0)
 *   k a <cw> <ccw>   : Live-tune CW & CCW angle scales (default 1.0 1.0)
 *   k m <L> <R>      : Live-tune left/right motor balance (default 1.0 1.0)
 *   k i <blend>      : Set IMU redundancy blend (0.0=off, 0.2=20% sanity blend)
 *   c                : Reset pose to SW corner (22.5, 22.5) facing North (0 deg)
 *   r <0..3> <L|R>   : Go to Left (L/W) or Right (R/E) (or N/S) of rack 0..3
 *                      (0=BL, 1=TL, 2=BR, 3=TR, e.g. "r 0 L" or "r 3 R")
 *   g <x> <y>        : Drive to coordinate (x, y) in cm
 *   m <r1><s1><r2><s2>: Full 2-rack mission (e.g. "m 0 L 3 R")
 *   s [r]            : 180-deg dual-sensor (IR+PING) scan ('s r' prints raw table)
 *   p                : Print live cliff sensor readings
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inc/tm4c123gh6pm.h>
#include "ping.h"
#include "adc.h"
#include "uart.h"
#include "Timer.h"
#include "lcd.h"
#include "servo.h"
#include "button.h"
#include "open_interface.h"
#include "movement.h"
#include "i2c.h"
#include "BNO055.h"
#include "init_imu.h"
#include "navigation.h"

#define MAX_DIST_CM       150.0f
#define DETECT_RANGE_CM   95.0f   // Any fused reading < 95 cm counts as an object
#define EDGE_DROP_CM      22.0f   // Or a sudden drop > 22 cm
#define SERVO_STEP_MS     35

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ScanObject detected_objects[MAX_DETECTED_OBJECTS];
int object_count = 0;

volatile char uart_data = 0;

// Shared scan arrays read by navigation.c for corridor collision checking
float scan_ir_cm[SCAN_POINTS];
float scan_ping_cm[SCAN_POINTS];
float scan_fused_cm[SCAN_POINTS];

static float ping_raw_array[SCAN_POINTS];
static float ir_raw_array[SCAN_POINTS];

int gap_angle = 90;

static void clean_and_fuse_scan(void);
static void detect_objects_fused(void);
static void print_detected_objects(int verbose_raw);
static void uart_read_line(char *buf, int max_len);
static int  parse_side_char(char c, RackSide *out_side);
static void run_distance_test(oi_t *sensor, float target_cm);
static void run_angle_test(oi_t *sensor, float target_cw_deg);
static void print_calibration_status(oi_t *sensor);

int main(void)
{
    timer_init();
    adc_init();
    uart_init();
    ping_init();
    servo_init();
    lcd_init();
    button_init();

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    oi_setMotorCalibration((double)DEFAULT_MOTOR_CAL_L, (double)DEFAULT_MOTOR_CAL_R);

    // Initialize BNO055 in gyro+accel IMUPLUS mode (0x08) - NO magnetometer interference,
    // and NO blocking calibration dance! Used strictly as redundancy.
    init_imu();
    set_imu_mode(BNO055_OPERATION_MODE_IMUPLUS);
    timer_waitMillis(100);

    float test_h = 0.0f;
    imu_enabled = (bno055_convert_float_euler_h_deg(&test_h) == 0) ? 1 : 0;

    // Start in Bottom-Left (SW) corner (22.5, 22.5) facing NORTH (0 deg)
    nav_init_sw_corner();
    servo_move(90.0f);

    lcd_printf("SW Corner(22,22)\nFace NORTH IMU:%d\nWait UART cmd", imu_enabled);
    uart_sendStr("\r\n=== Warehouse Bot Ready (SW Corner 22.5,22.5 Facing NORTH) ===\r\n");
    uart_sendStr("Calibrate: 't d [cm]' | 't a [deg]' | 't c' | 'k d/a/m/i ...'\r\n");
    uart_sendStr("Navigate : 'c' | 'r <0..3> <L|R>' | 'g X Y' | 'm R1 S1 R2 S2' | 's [r]' | 'p'\r\n");

    char line[80];
    while (1)
    {
        uart_read_line(line, sizeof(line));
        if (line[0] == '\0')
        {
            continue;
        }

        char cmd = line[0];

        if (cmd == 't')
        {
            // Calibration & pre-run accuracy checks
            char sub = 0;
            float val = 0.0f;
            int n = sscanf(line + 1, " %c %f", &sub, &val);

            if (n >= 1 && (sub == 'd' || sub == 'D'))
            {
                run_distance_test(sensor_data, (n >= 2 && val > 0.0f) ? val : 100.0f);
            }
            else if (n >= 1 && (sub == 'a' || sub == 'A'))
            {
                run_angle_test(sensor_data, (n >= 2 && val != 0.0f) ? val : 90.0f);
            }
            else if (n >= 1 && (sub == 'c' || sub == 'C'))
            {
                print_calibration_status(sensor_data);
            }
            else
            {
                uart_sendStr("Usage: t d [cm] | t a [deg] | t c\r\n");
            }
        }
        else if (cmd == 'k')
        {
            // Live calibration adjustment without reflashing
            char sub = 0;
            float v1 = 0.0f, v2 = 0.0f;
            int n = sscanf(line + 1, " %c %f %f", &sub, &v1, &v2);

            if (n >= 2 && (sub == 'd' || sub == 'D') && v1 > 1.0f)
            {
                cal_mm_per_cm = v1;
                char out[64];
                sprintf(out, "SET cal_mm_per_cm = %.3f\r\n", cal_mm_per_cm);
                uart_sendStr(out);
            }
            else if (n >= 2 && (sub == 'a' || sub == 'A') && v1 > 0.2f)
            {
                cal_angle_scale_cw = v1;
                cal_angle_scale_ccw = (n >= 3 && v2 > 0.2f) ? v2 : v1;
                char out[80];
                sprintf(out, "SET cal_angle_scale CW=%.3f CCW=%.3f\r\n",
                        cal_angle_scale_cw, cal_angle_scale_ccw);
                uart_sendStr(out);
            }
            else if (n >= 3 && (sub == 'm' || sub == 'M'))
            {
                oi_setMotorCalibration((double)v1, (double)v2);
                char out[64];
                sprintf(out, "SET motor_cal L=%.3f R=%.3f\r\n", v1, v2);
                uart_sendStr(out);
            }
            else if (n >= 2 && (sub == 'i' || sub == 'I'))
            {
                if (v1 < 0.0f) v1 = 0.0f;
                if (v1 > 0.5f) v1 = 0.5f;
                cal_imu_blend = v1;
                char out[64];
                sprintf(out, "SET cal_imu_blend = %.2f\r\n", cal_imu_blend);
                uart_sendStr(out);
            }
            else
            {
                uart_sendStr("Usage: k d <mm/cm> | k a <cw> [ccw] | k m <L> <R> | k i <0..0.5>\r\n");
            }
        }
        else if (cmd == 'c')
        {
            nav_init_sw_corner();
            uart_sendStr("RESET_SW_CORNER (22.5, 22.5) bearing=0.0 (NORTH)\r\n");
            lcd_printf("SW Corner(22,22)\nFacing NORTH");
        }
        else if (cmd == 'g')
        {
            float gx = 0.0f, gy = 0.0f;
            if (sscanf(line + 1, "%f %f", &gx, &gy) == 2)
            {
                nav_drive_to_coord(sensor_data, gx, gy, 1, 0);
            }
            else
            {
                uart_sendStr("Usage: g <x_cm> <y_cm>\r\n");
            }
        }
        else if (cmd == 'r')
        {
            // Go to Left (L/W) or Right (R/E) (or N/S) of any of the 4 racks
            int rid = 0;
            char sc = 'L';
            RackSide side;
            if (sscanf(line + 1, "%d %c", &rid, &sc) == 2 &&
                rid >= 0 && rid <= 3 && parse_side_char(sc, &side))
            {
                nav_approach_rack(sensor_data, (RackId)rid, side);
            }
            else
            {
                uart_sendStr("Usage: r <0=BL|1=TL|2=BR|3=TR> <L|R|N|S> (e.g. r 0 L)\r\n");
            }
        }
        else if (cmd == 'm')
        {
            int r1 = 0, r2 = 3;
            char c1 = 'L', c2 = 'R';
            RackSide s1, s2;
            if (sscanf(line + 1, "%d %c %d %c", &r1, &c1, &r2, &c2) == 4 &&
                r1 >= 0 && r1 <= 3 && r2 >= 0 && r2 <= 3 &&
                parse_side_char(c1, &s1) && parse_side_char(c2, &s2))
            {
                nav_run_mission(sensor_data, (RackId)r1, s1, (RackId)r2, s2);
            }
            else
            {
                uart_sendStr("Usage: m <r1> <s1> <r2> <s2> (e.g. m 0 L 3 R)\r\n");
            }
        }
        else if (cmd == 's')
        {
            char opt = 0;
            int verbose = (sscanf(line + 1, " %c", &opt) == 1 && (opt == 'r' || opt == 'R'));
            scan_field(verbose);
        }
        else if (cmd == 'p')
        {
            char out[96];
            oi_update(sensor_data);
            sprintf(out, "CLIFF L:%u FL:%u FR:%u R:%u\r\n",
                    sensor_data->cliffLeftSignal,
                    sensor_data->cliffFrontLeftSignal,
                    sensor_data->cliffFrontRightSignal,
                    sensor_data->cliffRightSignal);
            uart_sendStr(out);
            lcd_printf("L:%u FL:%u\nFR:%u R:%u",
                       sensor_data->cliffLeftSignal,
                       sensor_data->cliffFrontLeftSignal,
                       sensor_data->cliffFrontRightSignal,
                       sensor_data->cliffRightSignal);
        }
    }
}

// =========================================================================
// Pre-Run Accuracy Tests (Distance & Angle Calibration)
// =========================================================================

static void run_distance_test(oi_t *sensor, float target_cm)
{
    char out[128];
    float imu_drift = 0.0f;

    sprintf(out, "TEST_DIST %.1f cm (cal_mm_per_cm=%.3f)...\r\n", target_cm, cal_mm_per_cm);
    uart_sendStr(out);
    lcd_printf("Dist Test:\n%.0f cm", target_cm);

    double meas_cm = move_forward_tracked(sensor, target_cm, 140, 0, &imu_drift);

    sprintf(out, "DONE enc_cm=%.1f flag=%d imu_drift=%.1f deg\r\n",
            meas_cm, flag, imu_drift);
    uart_sendStr(out);
    sprintf(out, "Measure tape! New scale = %.3f * (%.1f / actual_cm) -> 'k d <val>'\r\n",
            cal_mm_per_cm, target_cm);
    uart_sendStr(out);
    lcd_printf("Dist:%.1fcm F:%d\nDrift:%.1f deg", meas_cm, flag, imu_drift);
}

static void run_angle_test(oi_t *sensor, float target_cw_deg)
{
    char out[128];
    float enc_deg = 0.0f, imu_deg = 0.0f;

    sprintf(out, "TEST_ANGLE %.1f deg (+CW/-CCW, scale CW=%.3f CCW=%.3f)...\r\n",
            target_cw_deg, cal_angle_scale_cw, cal_angle_scale_ccw);
    uart_sendStr(out);
    lcd_printf("Angle Test:\n%.0f deg", target_cw_deg);

    float blended = turn_by_degrees(sensor, target_cw_deg, &enc_deg, &imu_deg);

    sprintf(out, "DONE enc=%.1f imu=%.1f blended=%.1f\r\n", enc_deg, imu_deg, blended);
    uart_sendStr(out);
    float old_s = (target_cw_deg >= 0.0f) ? cal_angle_scale_cw : cal_angle_scale_ccw;
    sprintf(out, "Check protractor! New scale = %.3f * (actual_deg / %.1f) -> 'k a <cw> <ccw>'\r\n",
            old_s, fabsf(target_cw_deg));
    uart_sendStr(out);
    lcd_printf("Enc:%.1f\nIMU:%.1f", enc_deg, imu_deg);
}

static void print_calibration_status(oi_t *sensor)
{
    char out[128];
    float h = 0.0f;
    int h_ok = imu_read_heading_safe(&h);

    oi_update(sensor);
    servo_move(90.0f);
    timer_waitMillis(150);
    float p_cm = ping_read_cm();
    int ir_raw = adc_read();
    float ir_cm = (ir_raw > 0) ? (float)pow((double)((float)ir_raw / 18538.0f), -1.225) : MAX_DIST_CM;
    if (ir_cm > MAX_DIST_CM || ir_cm != ir_cm) ir_cm = MAX_DIST_CM;

    BotPose pos = nav_get_pose();

    sprintf(out, "POSE x=%.1f y=%.1f bearing=%.1f\r\n", pos.x, pos.y, pos.bearing_deg);
    uart_sendStr(out);
    sprintf(out, "CAL mm/cm=%.3f angCW=%.3f angCCW=%.3f motL=%.2f motR=%.2f imuBlend=%.2f\r\n",
            cal_mm_per_cm, cal_angle_scale_cw, cal_angle_scale_ccw,
            (float)oi_getMotorCalibrationLeft(), (float)oi_getMotorCalibrationRight(),
            cal_imu_blend);
    uart_sendStr(out);
    sprintf(out, "SENS PING=%.1fcm IR=%.1fcm(raw=%d) IMU=%s(%.1f) CLIFF=%u,%u,%u,%u\r\n",
            p_cm, ir_cm, ir_raw, h_ok ? "OK" : "OFF", h,
            sensor->cliffLeftSignal, sensor->cliffFrontLeftSignal,
            sensor->cliffFrontRightSignal, sensor->cliffRightSignal);
    uart_sendStr(out);
}

// =========================================================================
// UART & Parsing Helpers
// =========================================================================

static void uart_read_line(char *buf, int max_len)
{
    int idx = 0;
    while (idx < max_len - 1)
    {
        char ch = uart_receive();
        if (ch == '\r' || ch == '\n')
        {
            if (idx == 0) continue;
            break;
        }
        buf[idx++] = ch;
    }
    buf[idx] = '\0';
}

// Accepts L/W (Left/West), R/E (Right/East), N (North), S (South)
static int parse_side_char(char c, RackSide *out_side)
{
    switch (c)
    {
        case 'N': case 'n': *out_side = SIDE_NORTH; return 1;
        case 'E': case 'e': case 'R': case 'r': *out_side = SIDE_EAST; return 1;
        case 'S': case 's': *out_side = SIDE_SOUTH; return 1;
        case 'W': case 'w': case 'L': case 'l': *out_side = SIDE_WEST; return 1;
        default: return 0;
    }
}

// =========================================================================
// Dual-Sensor (IR + PING) Median-Filtered Scan & Object Detection
// =========================================================================

static float median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

void scan_field(int verbose_raw)
{
    int i, j;

    servo_move(0.0f);
    timer_waitMillis(300);

    for (i = 0; i <= 180; i += 2)
    {
        int idx = i / 2;
        servo_move((float)i);
        timer_waitMillis(SERVO_STEP_MS);

        float ping_sum = 0.0f;
        int ping_valid = 0;
        float ir_sum = 0.0f;

        for (j = 0; j < 3; j++)
        {
            int p = ping_read();
            if (p > 0)
            {
                ping_sum += (float)p;
                ping_valid++;
            }
            ir_sum += (float)adc_read();
        }

        ping_raw_array[idx] = (ping_valid > 0) ? (ping_sum / (float)ping_valid) : -1.0f;
        ir_raw_array[idx]   = ir_sum / 3.0f;
    }

    servo_move(90.0f);

    clean_and_fuse_scan();
    detect_objects_fused();
    print_detected_objects(verbose_raw);
}

static void clean_and_fuse_scan(void)
{
    float ir_tmp[SCAN_POINTS];
    float ping_tmp[SCAN_POINTS];
    int i;

    for (i = 0; i < SCAN_POINTS; i++)
    {
        // IR raw -> cm
        float iv = MAX_DIST_CM;
        if (ir_raw_array[i] > 50.0f)
        {
            iv = (float)pow((double)(ir_raw_array[i] / 18538.0f), -1.225);
            if (iv > MAX_DIST_CM || iv < 4.0f || iv != iv) iv = MAX_DIST_CM;
        }
        ir_tmp[i] = iv;

        // PING ticks -> cm
        float pv = MAX_DIST_CM;
        if (ping_raw_array[i] > 0.0f)
        {
            double pulse_ms = (double)ping_raw_array[i] / 16000.0;
            double d = (pulse_ms * 34.3) / 2.0 - 4.0;
            if (d < 4.0) d = 4.0;
            if (d > MAX_DIST_CM) d = MAX_DIST_CM;
            pv = (float)d;
        }
        ping_tmp[i] = pv;
    }

    // 3-point median filter preserves sharp thin pillars while killing single spikes
    for (i = 0; i < SCAN_POINTS; i++)
    {
        int prev = (i > 0) ? (i - 1) : 0;
        int next = (i < SCAN_POINTS - 1) ? (i + 1) : (SCAN_POINTS - 1);

        scan_ir_cm[i]   = median3(ir_tmp[prev],   ir_tmp[i],   ir_tmp[next]);
        scan_ping_cm[i] = median3(ping_tmp[prev], ping_tmp[i], ping_tmp[next]);

        // Fuse PING (primary, clean & accurate) + IR (only as ultra-close backup):
        // Lab scan showed IR produces phantom 70-85 cm spikes in open space, while
        // PING accurately measures 49-56 cm objects and 131-150 cm open space.
        float pg_use = scan_ping_cm[i];
        float ir_use = MAX_DIST_CM;
        if (scan_ir_cm[i] < 48.0f && (pg_use < 95.0f || pg_use >= (MAX_DIST_CM - 5.0f)))
        {
            ir_use = scan_ir_cm[i];
        }
        scan_fused_cm[i] = (ir_use < pg_use) ? ir_use : pg_use;
    }
}

static void record_object(int start_idx, int end_idx)
{
    int start_angle = start_idx * 2;
    int end_angle   = end_idx * 2;
    int radial_w    = end_angle - start_angle;

    // Require at least 4 degrees (>= 3 samples) to ignore stray noise
    if (radial_w < 4 || object_count >= MAX_DETECTED_OBJECTS)
    {
        return;
    }

    int center_angle = (start_angle + end_angle) / 2;

    // Use minimum fused distance across the object's interior so we never underestimate proximity
    float min_d = MAX_DIST_CM;
    int k;
    for (k = start_idx; k <= end_idx; k++)
    {
        if (scan_fused_cm[k] < min_d)
        {
            min_d = scan_fused_cm[k];
        }
    }

    detected_objects[object_count].id           = object_count + 1;
    detected_objects[object_count].start_angle  = start_angle;
    detected_objects[object_count].end_angle    = end_angle;
    detected_objects[object_count].center_angle = center_angle;
    detected_objects[object_count].radial_width = radial_w;
    detected_objects[object_count].distance     = min_d;

    float theta_rad = (radial_w * (float)M_PI) / 180.0f;
    detected_objects[object_count].linear_width = 2.0f * min_d * sinf(theta_rad / 2.0f);

    object_count++;
}

static void detect_objects_fused(void)
{
    object_count = 0;
    int on_object = 0;
    int start_idx = 0;
    float obj_ref_dist = MAX_DIST_CM;
    int i;

    for (i = 0; i < SCAN_POINTS; i++)
    {
        float d = scan_fused_cm[i];
        float prev_d = (i > 0) ? scan_fused_cm[i - 1] : MAX_DIST_CM;

        if (!on_object)
        {
            // Enter object if within DETECT_RANGE_CM OR sharp inward drop
            if (d < DETECT_RANGE_CM || (prev_d - d) > EDGE_DROP_CM)
            {
                on_object = 1;
                start_idx = i;
                obj_ref_dist = d;
            }
        }
        else
        {
            // Split if a second closer/farther object starts abruptly, or exit if open space
            if (d >= (DETECT_RANGE_CM + 10.0f) || (d - obj_ref_dist) > EDGE_DROP_CM)
            {
                record_object(start_idx, i - 1);
                on_object = 0;
            }
            else if ((obj_ref_dist - d) > EDGE_DROP_CM)
            {
                // Distinct closer object overlapped in front -> split cleanly
                record_object(start_idx, i - 1);
                start_idx = i;
                obj_ref_dist = d;
            }
            else
            {
                // Smoothly track object surface distance
                if (d < obj_ref_dist) obj_ref_dist = d;
            }
        }
    }

    if (on_object)
    {
        record_object(start_idx, SCAN_POINTS - 1);
    }
}

static void print_detected_objects(int verbose_raw)
{
    char out[96];

    if (verbose_raw)
    {
        int i;
        uart_sendStr("Deg   IR(cm)  PING(cm)  Fused(cm)\r\n");
        for (i = 0; i < SCAN_POINTS; i += 2)
        {
            sprintf(out, "%-4d  %-6.1f  %-8.1f  %-6.1f\r\n",
                    i * 2, scan_ir_cm[i], scan_ping_cm[i], scan_fused_cm[i]);
            uart_sendStr(out);
        }
    }

    uart_sendStr("Obj#  Angle  Dist(cm)  AngW  LinW(cm)\r\n");

    int biggest_gap = 0;
    int gap_center = 90;
    int prev_end = 0;
    int i;

    for (i = 0; i < object_count; i++)
    {
        sprintf(out, "%-4d  %-5d  %-8.1f  %-4d  %-6.1f\r\n",
                detected_objects[i].id,
                detected_objects[i].center_angle,
                detected_objects[i].distance,
                detected_objects[i].radial_width,
                detected_objects[i].linear_width);
        uart_sendStr(out);

        int gap = detected_objects[i].start_angle - prev_end;
        if (gap > biggest_gap)
        {
            biggest_gap = gap;
            gap_center = prev_end + (gap / 2);
        }
        prev_end = detected_objects[i].end_angle;
    }

    int end_gap = 180 - prev_end;
    if (end_gap > biggest_gap)
    {
        biggest_gap = end_gap;
        gap_center = prev_end + (end_gap / 2);
    }

    gap_angle = (object_count == 0) ? 90 : gap_center;

    int hit_ang = -1;
    float clear_fwd = nav_check_front_corridor_cm(100.0f, &hit_ang);
    sprintf(out, "GAP %ddeg (w=%d) | CORRIDOR_CLEAR=%.1fcm (hit_ang=%d)\r\n",
            gap_angle, biggest_gap, clear_fwd, hit_ang);
    uart_sendStr(out);
}
