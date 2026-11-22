/*
 * final_main / main.c
 *
 * Assisted Manual Control for the Autonomous Warehouse Bot (CPRE 288).
 * Keeps the familiar lab3_template structure (single-key PuTTY controls,
 * scan_field, clean_data, object_detect, print_detected_objects, socketFix)
 * while adding:
 *   - Automatic safety stops on Bumpers, White Walls, Black Holes, & Final Destination
 *   - Live BNO055 IMUPLUS (0x08, gyro+accel) compass heading & estimated (X, Y) HUD
 *   - Fast ~3.5 s PING-primary 180-deg scan + 1-line ASCII radar bar
 *   - Pickup & Delivery confirmation alerts + Roomba audio chimes
 *
 * Single-Key PuTTY Controls (no Enter key needed):
 *   q / w / e : Drive forward 5 cm / 20 cm / 45 cm (auto-stops on any hazard!)
 *   s         : Drive backward 10 cm
 *   a / d     : Turn Left / Right 15 deg
 *   z / c     : Turn Left / Right 90 deg
 *   x         : Turn around 180 deg
 *   m         : Fast 180-deg scan (ASCII radar + object table + biggest gap)
 *   i         : Status info (IMU heading, pos, front PING, all 4 cliff sensors)
 *   p         : Confirm Rack Pickup (chime + counter)
 *   f         : Confirm Drop-off / Order Complete (victory song on #2)
 *   r         : Reset heading = 0 deg (North) & pos = (22.5, 22.5) SW corner
 *   h         : Show controls help
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "Timer.h"
#include "uart.h"
#include "movement.h"
#include "adc.h"
#include "ping.h"
#include "servo.h"
#include "lcd.h"
#include "button.h"
#include "i2c.h"
#include "bno055.h"
#include "init_imu.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_OBJECTS     10
#define MAX_DIST_CM     150.0f
#define DETECT_RANGE_CM 95.0f
#define EDGE_DROP_CM    22.0f
#define SERVO_STEP_MS   35
#define DRIVE_SPEED     140

typedef struct {
    int id;
    int start_angle;
    int end_angle;
    int center_angle;
    float distance;
    int radial_width;
    float linear_width;
} Object;

Object detected_objects[MAX_OBJECTS];
int object_count = 0;

float sensor_data_array2[91];
float sensor_data_array2_cleaned[91];
float ir_data_array[91];
float ir_data_array_clean[91];

volatile char uart_data = 0;

// Pose & Mission Tracking (for blind-pilot HUD)
static float imu_zero_deg = 0.0f;   // IMU reading when facing 0 deg (North/start)
static float bot_x_cm     = 22.5f;  // Default SW corner X (cm)
static float bot_y_cm     = 22.5f;  // Default SW corner Y (cm)
static float bot_hdg_deg  = 0.0f;   // 0=N, 90=E, 180=S, 270=W

static int   items_picked    = 0;
static int   items_delivered = 0;
static int   dest_found      = 0;
static float dest_x_cm       = 0.0f;
static float dest_y_cm       = 0.0f;

// Function prototypes (matching lab3_template convention)
void cyBot_send_string(const char *str);
void scan_field(void);
void clean_data(void);
void object_detect(float distance, int current_angle);
void print_detected_objects(void);
void socketFix(void);

// Manual-pilot helpers
static void  print_help(void);
static float get_heading_deg(void);
static const char *heading_to_compass(float deg);
static void  update_pose_from_move(float signed_cm);
static void  drive_manual(oi_t *sensor, double cm);
static void  turn_manual(oi_t *sensor, float delta_cw_deg);
static void  print_hud(const char *status_msg);
static void  print_full_status(oi_t *sensor);
static void  play_beep_pickup(void);
static void  play_beep_dest(void);
static void  play_song_complete(void);

int main(void) {
    // Configure all devices (same order as working project)
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

    // Initialize BNO055 in gyro+accel IMUPLUS mode (0x08) - no magnetometer jumps!
    init_imu();
    set_imu_mode(BNO055_OPERATION_MODE_IMUPLUS);
    timer_waitMillis(100);

    float h0 = 0.0f;
    if (bno055_convert_float_euler_h_deg(&h0) == 0) {
        imu_enabled = 1;
        imu_zero_deg = normalize_deg(h0);
    } else {
        imu_enabled = 0;
    }

    servo_move(90.0f);
    bot_x_cm = 22.5f;
    bot_y_cm = 22.5f;
    bot_hdg_deg = 0.0f;

    lcd_printf("MANUAL PILOT\nIMU:%d Ready!\nWait PuTTY...", imu_enabled);
    cyBot_send_string("\r\n========================================\r\n");
    cyBot_send_string("  WAREHOUSE BOT - MANUAL PILOT READY!\r\n");
    cyBot_send_string("========================================\r\n");
    print_help();
    print_hud("READY");
    socketFix();

    char got_Byte = 0;

    while (1) {
        if (uart_receive_nonblock(&got_Byte)) {
            if (got_Byte == '\r' || got_Byte == '\n') {
                continue;
            }

            if (got_Byte == 'q' || got_Byte == 'Q') {
                drive_manual(sensor_data, 5.0);
                socketFix();
            } else if (got_Byte == 'w' || got_Byte == 'W') {
                drive_manual(sensor_data, 20.0);
                socketFix();
            } else if (got_Byte == 'e' || got_Byte == 'E') {
                drive_manual(sensor_data, 45.0);
                socketFix();
            } else if (got_Byte == 's' || got_Byte == 'S') {
                double moved = move_backward(sensor_data, 10.0, DRIVE_SPEED);
                update_pose_from_move(-(float)moved);
                print_hud("BACKED UP 10cm");
                socketFix();
            } else if (got_Byte == 'a' || got_Byte == 'A') {
                turn_manual(sensor_data, -15.0f);
                socketFix();
            } else if (got_Byte == 'd' || got_Byte == 'D') {
                turn_manual(sensor_data, 15.0f);
                socketFix();
            } else if (got_Byte == 'z' || got_Byte == 'Z') {
                turn_manual(sensor_data, -90.0f);
                socketFix();
            } else if (got_Byte == 'c' || got_Byte == 'C') {
                turn_manual(sensor_data, 90.0f);
                socketFix();
            } else if (got_Byte == 'x' || got_Byte == 'X') {
                turn_manual(sensor_data, 180.0f);
                socketFix();
            } else if (got_Byte == 'm' || got_Byte == 'M') {
                lcd_printf("Scanning 180...");
                scan_field();
                print_hud("SCAN DONE");
                socketFix();
            } else if (got_Byte == 'i' || got_Byte == 'I') {
                print_full_status(sensor_data);
                socketFix();
            } else if (got_Byte == 'p' || got_Byte == 'P') {
                char msg[96];
                items_picked++;
                play_beep_pickup();
                sprintf(msg, "\r\n*** RACK PICKUP #%d CONFIRMED at (%.0f, %.0f) ***\r\n",
                        items_picked, bot_x_cm, bot_y_cm);
                cyBot_send_string(msg);
                lcd_printf("PICKUP #%d!\nPos:%.0f,%.0f", items_picked, bot_x_cm, bot_y_cm);
                socketFix();
            } else if (got_Byte == 'f' || got_Byte == 'F') {
                char msg[96];
                items_delivered++;
                if (items_delivered >= 2) {
                    play_song_complete();
                    sprintf(msg, "\r\n*** ORDER COMPLETE! (2/2 DELIVERED) ***\r\n");
                    lcd_printf("ORDER COMPLETE!\n2/2 Delivered!");
                } else {
                    play_beep_dest();
                    sprintf(msg, "\r\n*** DROP-OFF #%d CONFIRMED! Go to Rack #2 ***\r\n",
                            items_delivered);
                    lcd_printf("DROP-OFF #%d!\nGo to Rack #2", items_delivered);
                }
                cyBot_send_string(msg);
                socketFix();
            } else if (got_Byte == 'r' || got_Byte == 'R') {
                float cur = 0.0f;
                if (imu_read_heading_safe(&cur)) {
                    imu_zero_deg = cur;
                }
                bot_x_cm = 22.5f;
                bot_y_cm = 22.5f;
                bot_hdg_deg = 0.0f;
                items_picked = 0;
                items_delivered = 0;
                dest_found = 0;
                cyBot_send_string("\r\n--- RESET: SW Corner (22.5, 22.5), HDG = 0.0 (N) ---\r\n");
                print_hud("RESET");
                socketFix();
            } else if (got_Byte == 'h' || got_Byte == 'H') {
                print_help();
                socketFix();
            }
        }
    }
}

// =========================================================================
// Manual Driving & Turning with Automatic Safety Stops + HUD
// =========================================================================

static void drive_manual(oi_t *sensor, double cm) {
    char msg[128];
    double moved = move_forward_until_stop(sensor, cm, DRIVE_SPEED);
    update_pose_from_move((float)moved);

    if (flag == MOVE_FLAG_NONE) {
        sprintf(msg, "MOVED %.1fcm", moved);
        print_hud(msg);
        return;
    }

    if (flag == MOVE_FLAG_DESTINATION) {
        // Stop right on the destination mat (do NOT back up!)
        dest_found = 1;
        dest_x_cm = bot_x_cm;
        dest_y_cm = bot_y_cm;
        play_beep_dest();
        sprintf(msg, "\r\n*** FINAL DESTINATION FOUND at (%.0f, %.0f) cm! ***\r\n",
                dest_x_cm, dest_y_cm);
        cyBot_send_string(msg);
        lcd_printf("DESTINATION!\n(%.0f, %.0f) cm", dest_x_cm, dest_y_cm);
        print_hud("ON DESTINATION!");
        return;
    }

    // Any hazard (bump, white wall, or black hole): announce & back off 6 cm safely
    const char *hazard_str = "HAZARD";
    switch (flag) {
        case MOVE_FLAG_BUMP_LEFT:  hazard_str = "!! BUMP LEFT (Fallen Box) !!";  break;
        case MOVE_FLAG_BUMP_RIGHT: hazard_str = "!! BUMP RIGHT (Fallen Box) !!"; break;
        case MOVE_FLAG_BUMP_BOTH:  hazard_str = "!! BUMP FRONT (Fallen Box) !!"; break;
        case MOVE_FLAG_WALL_LEFT:  hazard_str = "!! WHITE WALL LEFT !!";         break;
        case MOVE_FLAG_WALL_RIGHT: hazard_str = "!! WHITE WALL RIGHT !!";        break;
        case MOVE_FLAG_WALL_FRONT: hazard_str = "!! WHITE WALL FRONT !!";        break;
        case MOVE_FLAG_HOLE_LEFT:  hazard_str = "!! BLACK HOLE LEFT !!";         break;
        case MOVE_FLAG_HOLE_RIGHT: hazard_str = "!! BLACK HOLE RIGHT !!";        break;
        case MOVE_FLAG_HOLE_FRONT: hazard_str = "!! BLACK HOLE FRONT !!";        break;
        default: break;
    }

    sprintf(msg, "\r\n%s (stopped at %.1f cm -> backing up 6 cm)\r\n", hazard_str, moved);
    cyBot_send_string(msg);
    lcd_printf("%s", hazard_str);

    double backed = move_backward(sensor, 6.0, DRIVE_SPEED);
    update_pose_from_move(-(float)backed);
    print_hud(hazard_str);
}

static void turn_manual(oi_t *sensor, float delta_cw_deg) {
    char msg[64];
    float enc_d = 0.0f, imu_d = 0.0f;
    float turned = turn_by_degrees(sensor, delta_cw_deg, &enc_d, &imu_d);

    bot_hdg_deg = normalize_deg(bot_hdg_deg + turned);
    sprintf(msg, "TURN %+.0fdeg (enc:%.1f imu:%.1f)", delta_cw_deg, enc_d, imu_d);
    print_hud(msg);
}

static float get_heading_deg(void) {
    float raw_h = 0.0f;
    if (imu_read_heading_safe(&raw_h)) {
        bot_hdg_deg = normalize_deg(raw_h - imu_zero_deg);
    }
    return bot_hdg_deg;
}

static const char *heading_to_compass(float deg) {
    deg = normalize_deg(deg);
    if (deg >= 337.5f || deg < 22.5f)  return "N";
    if (deg < 67.5f)                   return "NE";
    if (deg < 112.5f)                  return "E";
    if (deg < 157.5f)                  return "SE";
    if (deg < 202.5f)                  return "S";
    if (deg < 247.5f)                  return "SW";
    if (deg < 292.5f)                  return "W";
    return "NW";
}

static void update_pose_from_move(float signed_cm) {
    float hdg = get_heading_deg();
    float rad = hdg * ((float)M_PI / 180.0f);
    // 0 deg = +Y (North), 90 deg = +X (East)
    bot_x_cm += signed_cm * sinf(rad);
    bot_y_cm += signed_cm * cosf(rad);
}

static void print_hud(const char *status_msg) {
    char out[140];
    float hdg = get_heading_deg();
    const char *comp = heading_to_compass(hdg);

    servo_move(90.0f);
    timer_waitMillis(60);
    float front_cm = ping_read_cm();

    if (front_cm < 0.0f) {
        sprintf(out, "[HDG:%5.1f (%-2s) | POS:(%3.0f,%3.0f) | FRONT: >150cm | %s]\r\n",
                hdg, comp, bot_x_cm, bot_y_cm, status_msg);
    } else {
        sprintf(out, "[HDG:%5.1f (%-2s) | POS:(%3.0f,%3.0f) | FRONT:%5.1fcm | %s]\r\n",
                hdg, comp, bot_x_cm, bot_y_cm, front_cm, status_msg);
    }
    cyBot_send_string(out);

    lcd_printf("H:%.0f(%s) P:%.0f,%.0f\nF:%.0fcm %s",
               hdg, comp, bot_x_cm, bot_y_cm,
               (front_cm < 0.0f) ? 150.0f : front_cm, status_msg);
}

static void print_full_status(oi_t *sensor) {
    char out[140];
    oi_update(sensor);

    float hdg = get_heading_deg();
    const char *comp = heading_to_compass(hdg);

    servo_move(90.0f);
    timer_waitMillis(120);
    float front_ping = ping_read_cm();
    int ir_raw = adc_read();
    float ir_cm = (ir_raw > 50) ? (float)pow((double)((float)ir_raw / 18538.0f), -1.225) : MAX_DIST_CM;
    if (ir_cm > MAX_DIST_CM || ir_cm != ir_cm) ir_cm = MAX_DIST_CM;

    cyBot_send_string("\r\n--- ROBOT STATUS ---\r\n");
    sprintf(out, "Heading : %.1f deg (%s) [IMU:%s]\r\n",
            hdg, comp, imu_enabled ? "ON" : "OFF");
    cyBot_send_string(out);
    sprintf(out, "Est Pos : (%.1f, %.1f) cm\r\n", bot_x_cm, bot_y_cm);
    cyBot_send_string(out);
    sprintf(out, "Front   : PING=%.1f cm | IR=%.1f cm (raw=%d)\r\n",
            front_ping, ir_cm, ir_raw);
    cyBot_send_string(out);
    sprintf(out, "Cliffs  : L=%u  FL=%u  FR=%u  R=%u\r\n",
            sensor->cliffLeftSignal, sensor->cliffFrontLeftSignal,
            sensor->cliffFrontRightSignal, sensor->cliffRightSignal);
    cyBot_send_string(out);
    sprintf(out, "Mission : Picked=%d/2 | Delivered=%d/2 | Dest=%s (%.0f,%.0f)\r\n",
            items_picked, items_delivered,
            dest_found ? "SAVED" : "not yet", dest_x_cm, dest_y_cm);
    cyBot_send_string(out);
}

static void print_help(void) {
    cyBot_send_string("Keys: [q]=5cm [w]=20cm [e]=45cm [s]=back10cm\r\n");
    cyBot_send_string("      [a/d]=L/R 15deg  [z/c]=L/R 90deg  [x]=180deg\r\n");
    cyBot_send_string("      [m]=Scan 180+Radar [i]=Full Status [p]=Pickup [f]=Finish [r]=Reset\r\n");
}

// =========================================================================
// Fast 180-Degree Scan + ASCII Radar + Object Table (lab3_template style!)
// =========================================================================

static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

void scan_field(void) {
    int i, j;

    servo_move(0.0f);
    timer_waitMillis(300);

    for (i = 0; i <= 180; i += 2) {
        int idx = i / 2;
        servo_move((float)i);
        timer_waitMillis(SERVO_STEP_MS);

        float ping_sum = 0.0f;
        int ping_valid = 0;
        float ir_sum = 0.0f;

        for (j = 0; j < 3; j++) {
            int p = ping_read();
            if (p > 0) {
                ping_sum += (float)p;
                ping_valid++;
            }
            ir_sum += (float)adc_read();
        }

        sensor_data_array2[idx] = (ping_valid > 0) ? (ping_sum / (float)ping_valid) : -1.0f;
        ir_data_array[idx]      = ir_sum / 3.0f;
    }

    servo_move(90.0f);

    clean_data();

    object_count = 0;
    for (i = 0; i < 91; i++) {
        object_detect(sensor_data_array2_cleaned[i], i * 2);
    }

    print_detected_objects();
}

void clean_data(void) {
    float ping_tmp[91];
    float ir_tmp[91];
    int i;

    for (i = 0; i < 91; i++) {
        // PING ticks -> cm (primary sensor, verified accurate in lab)
        float pv = MAX_DIST_CM;
        if (sensor_data_array2[i] > 0.0f) {
            double pulse_ms = (double)sensor_data_array2[i] / 16000.0;
            double distance_cm = (pulse_ms * 34.3) / 2.0 - 4.0;
            if (distance_cm < 4.0) distance_cm = 4.0;
            if (distance_cm > MAX_DIST_CM) distance_cm = MAX_DIST_CM;
            pv = (float)distance_cm;
        }
        ping_tmp[i] = pv;

        // IR raw -> cm (kept for close backup & telemetry)
        float iv = MAX_DIST_CM;
        if (ir_data_array[i] > 50.0f) {
            iv = (float)pow((double)(ir_data_array[i] / 18538.0f), -1.225);
            if (iv > MAX_DIST_CM || iv < 4.0f || iv != iv) iv = MAX_DIST_CM;
        }
        ir_tmp[i] = iv;
    }

    // 3-point median filter kills spikes without erasing thin pillars
    for (i = 0; i < 91; i++) {
        int prev = (i > 0) ? (i - 1) : 0;
        int next = (i < 90) ? (i + 1) : 90;
        sensor_data_array2_cleaned[i] = median3(ping_tmp[prev], ping_tmp[i], ping_tmp[next]);
        ir_data_array_clean[i]        = median3(ir_tmp[prev],   ir_tmp[i],   ir_tmp[next]);
    }
}

static void save_object(int start_angle, int end_angle) {
    int radial_w = end_angle - start_angle;
    if (radial_w < 4 || object_count >= MAX_OBJECTS) {
        return;
    }

    int center_angle = (start_angle + end_angle) / 2;
    int s_idx = start_angle / 2;
    int e_idx = end_angle / 2;
    float min_d = MAX_DIST_CM;
    int k;
    for (k = s_idx; k <= e_idx; k++) {
        if (sensor_data_array2_cleaned[k] < min_d) {
            min_d = sensor_data_array2_cleaned[k];
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

void object_detect(float distance, int current_angle) {
    static int on_object = 0;
    static int start_angle = 0;
    static int prev_angle = 0;
    static float prev_dist = MAX_DIST_CM;
    static float obj_ref = MAX_DIST_CM;

    if (current_angle == 0) {
        on_object = (distance < DETECT_RANGE_CM) ? 1 : 0;
        start_angle = 0;
        prev_angle = 0;
        prev_dist = distance;
        obj_ref = distance;
        return;
    }

    if (!on_object) {
        if (distance < DETECT_RANGE_CM || (prev_dist - distance) > EDGE_DROP_CM) {
            on_object = 1;
            start_angle = current_angle;
            obj_ref = distance;
        }
    } else {
        if (distance >= (DETECT_RANGE_CM + 10.0f) || (distance - obj_ref) > EDGE_DROP_CM) {
            on_object = 0;
            save_object(start_angle, prev_angle);
        } else if ((obj_ref - distance) > EDGE_DROP_CM) {
            // Closer object overlapping in front -> split
            save_object(start_angle, prev_angle);
            start_angle = current_angle;
            obj_ref = distance;
        } else if (distance < obj_ref) {
            obj_ref = distance;
        }
    }

    prev_dist = distance;
    prev_angle = current_angle;

    if (current_angle == 180 && on_object) {
        on_object = 0;
        save_object(start_angle, prev_angle);
    }
}

void print_detected_objects(void) {
    char out[100];
    int i;

    // 1. Visual 1-line ASCII Radar from LEFT (180 deg) to RIGHT (0 deg)
    //    '#' = <30cm (VERY CLOSE)  'o' = 30-65cm  '.' = 65-95cm  '_' = CLEAR (>95cm)
    cyBot_send_string("\r\nRADAR (Left 180 -> Front 90 -> Right 0): [#<30 o<65 .<95 _clear]\r\n");
    char radar[52];
    int rpos = 0;
    for (i = 90; i >= 0; i -= 2) { // 46 chars covering 180..0 in 4-deg bins
        float d = sensor_data_array2_cleaned[i];
        if (d < 30.0f)      radar[rpos++] = '#';
        else if (d < 65.0f) radar[rpos++] = 'o';
        else if (d < 95.0f) radar[rpos++] = '.';
        else                radar[rpos++] = '_';
    }
    radar[rpos] = '\0';
    sprintf(out, "[%s]\r\n", radar);
    cyBot_send_string(out);
    cyBot_send_string(" 180(L)       135        90(F)       45        0(R)\r\n");

    // 2. Object Table (same format as lab3_template)
    cyBot_send_string("Object#  Angle  Dist(cm)  AngW   LinW(cm)  Type?\r\n");

    int biggest_gap = 0;
    int gap_center = 90;
    int prev_end = 0;

    for (i = 0; i < object_count; i++) {
        const char *guess = "Pillar";
        if (detected_objects[i].linear_width >= 22.0f) guess = "RACK";
        else if (detected_objects[i].linear_width >= 11.0f) guess = "Wide";

        sprintf(out, "%-7d  %-5d  %-8.1f  %-5d  %-8.1f  %s\r\n",
                detected_objects[i].id,
                detected_objects[i].center_angle,
                detected_objects[i].distance,
                detected_objects[i].radial_width,
                detected_objects[i].linear_width,
                guess);
        cyBot_send_string(out);

        int gap = detected_objects[i].start_angle - prev_end;
        if (gap > biggest_gap) {
            biggest_gap = gap;
            gap_center = prev_end + (gap / 2);
        }
        prev_end = detected_objects[i].end_angle;
    }

    int end_gap = 180 - prev_end;
    if (end_gap > biggest_gap) {
        biggest_gap = end_gap;
        gap_center = prev_end + (end_gap / 2);
    }

    if (object_count == 0) {
        cyBot_send_string("No objects within 95 cm - FRONT CLEAR!\r\n");
    } else {
        sprintf(out, "Biggest open gap: %d deg (width %d deg)\r\n", gap_center, biggest_gap);
        cyBot_send_string(out);
    }
}

void cyBot_send_string(const char *str) {
    while (*str != '\0') {
        uart_sendChar(*str);
        str++;
    }
}

void socketFix(void) {
    cyBot_send_string("end\n");
}

// =========================================================================
// Audio Alerts (Bonus Capability!)
// =========================================================================

static void play_beep_pickup(void) {
    unsigned char notes[2] = { 64, 71 };
    unsigned char dur[2]   = { 12, 20 };
    oi_loadSong(0, 2, notes, dur);
    oi_play_song(0);
}

static void play_beep_dest(void) {
    unsigned char notes[3] = { 67, 72, 76 };
    unsigned char dur[3]   = { 12, 12, 24 };
    oi_loadSong(1, 3, notes, dur);
    oi_play_song(1);
}

static void play_song_complete(void) {
    unsigned char notes[5] = { 60, 64, 67, 72, 76 };
    unsigned char dur[5]   = { 12, 12, 12, 16, 32 };
    oi_loadSong(2, 5, notes, dur);
    oi_play_song(2);
}
