/*
 * main.c
 *
 *  Created on: Oct 29, 2026
 *  Updated on: Nov 22, 2026
 *      Authors: carlos28, jjaritz
 *
 * UART Commands (via PuTTY / GUI):
 *   c            : Set pose to SW corner (22.5, 22.5) for Milestone 1 testing
 *   g <x> <y>    : Drive to coordinate (x, y) in cm (e.g. "g 140 130")
 *   l            : Localize automatically using two orthogonal white walls
 *   r <id> <dir> : Approach rack (id 0=BL 1=TL 2=BR 3=TR, dir N/E/S/W, e.g. "r 0 W")
 *   m <r1> <s1> <r2> <s2> : Run full 2-rack mission (e.g. "m 0 W 3 S")
 *   s            : Run single 180-deg scan and print detected objects
 *   p            : Print live cliff sensor values (use to calibrate destination paper!)
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
#include <inc/tm4c123gh6pm.h>
#include "lcd.h"
#include "servo.h"
#include "button.h"
#include "open_interface.h"
#include "movement.h"
#include "i2c.h"
#include "BNO055.h"
#include "init_imu.h"
#include "navigation.h"

#define DELTA_THRESHOLD 35.0f
#define DELTA_ERROR     35.0f
#define MAX_DIST_CM     150.0f
#define SERVO_STEP_MS   40

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ScanObject detected_objects[MAX_DETECTED_OBJECTS];
int object_count = 0;

volatile char uart_data = 0;

static float ping_raw_array[91];
static float ping_cm_array[91];
static float ir_raw_array[91];
static float ir_cm_array[91];

int gap_angle = 90;

static void clean_data(void);
static void object_detect(float distance, int current_angle);
static void print_detected_objects(void);
static void wait_for_calibration(void);
static void uart_read_line(char *buf, int max_len);
static int  parse_side_char(char c, RackSide *out_side);

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

    init_imu();
    set_imu_mode(0x0C); // NDOF fused orientation mode

    // 1. Wait for BNO055 calibration
    wait_for_calibration();

    // 2. Point robot SOUTH and press SW1 to lock arena cardinal axes
    float south_target = 180.0f;
    while (button_getButton() != 1)
    {
        bno055_convert_float_euler_h_deg(&south_target);
        lcd_printf("Point SOUTH &\npress SW1\nHeading: %.1f", south_target);
        timer_waitMillis(50);
    }

    nav_init(south_target);
    // Default start pose = SW corner (safe for Milestone 1 corner testing)
    nav_set_pose(22.5f, 22.5f);

    timer_waitMillis(500);
    lcd_printf("Ready! S:%.1f\nWait UART cmd", south_target);
    uart_sendStr("\r\n=== Warehouse Bot Ready ===\r\n");
    uart_sendStr("Cmds: c | g X Y | l | r ID DIR | m R1 S1 R2 S2 | s | p\r\n");

    char line[64];
    while (1)
    {
        uart_read_line(line, sizeof(line));
        if (line[0] == '\0')
        {
            continue;
        }

        char cmd = line[0];

        if (cmd == 'c')
        {
            // Milestone 1 helper: place bot in bottom-left (SW) corner
            nav_init(south_target);
            nav_set_pose(22.5f, 22.5f);
            uart_sendStr("POSE_SET_SW_CORNER 22.5 22.5\r\n");
            lcd_printf("Pose: SW Corner\n(22.5, 22.5)");
        }
        else if (cmd == 'g')
        {
            // Drive to coordinate: "g <x> <y>"
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
        else if (cmd == 'l')
        {
            // Automatic two-wall localization
            nav_init(south_target);
            nav_localize_two_walls(sensor_data);
        }
        else if (cmd == 'r')
        {
            // Approach single rack side: "r <0..3> <N|E|S|W>"
            int rid = 0;
            char sc = 'W';
            RackSide side;
            if (sscanf(line + 1, "%d %c", &rid, &sc) == 2 &&
                rid >= 0 && rid <= 3 && parse_side_char(sc, &side))
            {
                nav_approach_rack(sensor_data, (RackId)rid, side);
            }
            else
            {
                uart_sendStr("Usage: r <0..3> <N|E|S|W>\r\n");
            }
        }
        else if (cmd == 'm')
        {
            // Full mission: "m <r1> <s1> <r2> <s2>"
            int r1 = 0, r2 = 3;
            char c1 = 'W', c2 = 'S';
            RackSide s1, s2;
            if (sscanf(line + 1, "%d %c %d %c", &r1, &c1, &r2, &c2) == 4 &&
                r1 >= 0 && r1 <= 3 && r2 >= 0 && r2 <= 3 &&
                parse_side_char(c1, &s1) && parse_side_char(c2, &s2))
            {
                nav_run_mission(sensor_data, (RackId)r1, s1, (RackId)r2, s2);
            }
            else
            {
                uart_sendStr("Usage: m <r1> <s1> <r2> <s2> (e.g. m 0 W 3 S)\r\n");
            }
        }
        else if (cmd == 's')
        {
            scan_field();
        }
        else if (cmd == 'p')
        {
            // Print live cliff sensors (useful to calibrate destination mat)
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

static void uart_read_line(char *buf, int max_len)
{
    int idx = 0;
    while (idx < max_len - 1)
    {
        char ch = uart_receive();
        if (ch == '\r' || ch == '\n')
        {
            if (idx == 0) continue; // Ignore stray CR/LF
            break;
        }
        buf[idx++] = ch;
    }
    buf[idx] = '\0';
}

static int parse_side_char(char c, RackSide *out_side)
{
    switch (c)
    {
        case 'N': case 'n': *out_side = SIDE_NORTH; return 1;
        case 'E': case 'e': *out_side = SIDE_EAST;  return 1;
        case 'S': case 's': *out_side = SIDE_SOUTH; return 1;
        case 'W': case 'w': *out_side = SIDE_WEST;  return 1;
        default: return 0;
    }
}

// Fast 180-degree IR + PING sweep (~3.5 s instead of 45 s)
void scan_field(void)
{
    int i, j;

    servo_move(0.0f);
    timer_waitMillis(350);

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

    clean_data();

    object_count = 0;
    for (i = 0; i < 91; i++)
    {
        object_detect(ir_cm_array[i], i * 2);
    }
    print_detected_objects();
}

static void clean_data(void)
{
    int i;
    for (i = 0; i < 91; i++)
    {
        float val = MAX_DIST_CM;
        if (ir_raw_array[i] > 0.0f)
        {
            val = (float)pow((double)(ir_raw_array[i] / 18538.0f), -1.225);
            if (val > MAX_DIST_CM || val != val)
            {
                val = MAX_DIST_CM;
            }
        }
        ir_cm_array[i] = val;
    }

    // Segment smoothing (bounds-safe!)
    float running_sum = ir_cm_array[0];
    int segment_start = 0;
    int count = 1;

    for (i = 1; i < 91; i++)
    {
        float avg = running_sum / (float)count;
        if (i + 2 < 91 &&
            fabsf(ir_cm_array[i + 1] - avg) > DELTA_ERROR &&
            fabsf(ir_cm_array[i + 2] - avg) > DELTA_ERROR)
        {
            int a;
            for (a = segment_start; a <= i; a++)
            {
                ir_cm_array[a] = avg;
            }
            segment_start = i + 1;
            running_sum = ir_cm_array[segment_start];
            count = 1;
        }
        else
        {
            running_sum += ir_cm_array[i];
            count++;
        }
    }

    // Flush final segment
    if (count > 0)
    {
        float avg = running_sum / (float)count;
        int a;
        for (a = segment_start; a < 91; a++)
        {
            ir_cm_array[a] = avg;
        }
    }

    // Convert PING ticks to cm (handle timeouts cleanly)
    for (i = 0; i < 91; i++)
    {
        if (ping_raw_array[i] <= 0.0f)
        {
            ping_cm_array[i] = MAX_DIST_CM;
        }
        else
        {
            double pulse_ms = (double)ping_raw_array[i] / 16000.0;
            double distance_cm = (pulse_ms * 34.3) / 2.0 - 4.0;
            if (distance_cm < 0.0) distance_cm = 0.0;
            if (distance_cm > MAX_DIST_CM) distance_cm = MAX_DIST_CM;
            ping_cm_array[i] = (float)distance_cm;
        }
    }
}

static void record_object(int start_angle, int end_angle)
{
    int radial_w = end_angle - start_angle;
    if (radial_w < 4 || object_count >= MAX_DETECTED_OBJECTS)
    {
        return;
    }

    int center_angle = (start_angle + end_angle) / 2;
    int cidx = center_angle / 2;
    int lo = (cidx > 0) ? (cidx - 1) : 0;
    int hi = (cidx < 90) ? (cidx + 1) : 90;

    // Average neighboring PING samples safely within [0, 90]
    float dist = (ping_cm_array[lo] + ping_cm_array[cidx] + ping_cm_array[hi]) / 3.0f;

    detected_objects[object_count].id           = object_count + 1;
    detected_objects[object_count].start_angle  = start_angle;
    detected_objects[object_count].end_angle    = end_angle;
    detected_objects[object_count].center_angle = center_angle;
    detected_objects[object_count].radial_width = radial_w;
    detected_objects[object_count].distance     = dist;

    float theta_rad = (radial_w * (float)M_PI) / 180.0f;
    detected_objects[object_count].linear_width = 2.0f * dist * sinf(theta_rad / 2.0f);

    object_count++;
}

static void object_detect(float distance, int current_angle)
{
    static int on_object = 0;
    static int start_angle = 0;
    static int prev_angle = 0;
    static float prev_dist = MAX_DIST_CM;

    if (current_angle == 0)
    {
        on_object = (distance < (MAX_DIST_CM - DELTA_THRESHOLD)) ? 1 : 0;
        start_angle = 0;
        prev_angle = 0;
        prev_dist = distance;
        return;
    }

    if (!on_object)
    {
        if ((prev_dist - distance) > DELTA_THRESHOLD)
        {
            on_object = 1;
            start_angle = current_angle;
        }
    }
    else
    {
        if ((distance - prev_dist) > DELTA_THRESHOLD)
        {
            on_object = 0;
            record_object(start_angle, prev_angle);
        }
    }

    prev_dist = distance;
    prev_angle = current_angle;

    if (current_angle == 180 && on_object)
    {
        on_object = 0;
        record_object(start_angle, prev_angle);
    }
}

static void print_detected_objects(void)
{
    char out[96];
    uart_sendStr("Object#  Angle  Dist(cm)  AngWidth  LinWidth(cm)\r\n");

    int biggest_gap = 0;
    int gap_center = 90;
    int prev_end = 0;
    int i;

    for (i = 0; i < object_count; i++)
    {
        sprintf(out, "%-7d  %-5d  %-8.1f  %-8d  %-8.1f\r\n",
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

    // Check trailing gap from last object to 180 deg
    int end_gap = 180 - prev_end;
    if (end_gap > biggest_gap)
    {
        biggest_gap = end_gap;
        gap_center = prev_end + (end_gap / 2);
    }

    gap_angle = (object_count == 0) ? 90 : gap_center;
    sprintf(out, "GAP_CENTER %d (width %d deg)\r\n", gap_angle, biggest_gap);
    uart_sendStr(out);
}

static void wait_for_calibration(void)
{
    Calibration cal;

    while (1)
    {
        cal = calibrate_imu();
        lcd_printf("G:%d M:%d A:%d S:%d\nErr:%d",
                   cal.gyro, cal.magn, cal.accl, cal.syst, get_error());

        if (cal.gyro == 3 && cal.magn == 3)
        {
            break;
        }

        // Override if indoor metal holds Mag at 2
        if (cal.gyro == 3 && button_getButton() == 1)
        {
            break;
        }

        timer_waitMillis(100);
    }

    lcd_printf("IMU Ready!");
    timer_waitMillis(500);
}
