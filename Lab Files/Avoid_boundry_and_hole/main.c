 /*
 * main.c
 *
 *  Created on: Oct 29, 2026
 *      Author: carlos28
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
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
//#include "i2c.h"
//#include "bno055.h"
//#include "init_imu.h"

//#define SOUTH_HEADING   180.0f
//#define HEADING_TOL     1.5f

#define DELTA_THRESHOLD 40.0f
#define M_PI 3.14159265358979323846
#define DELTA_ERROR 40.0f
#define ANGLE_ERROR 2
#define MAX_DIST_CM 150.0f

typedef struct {
    int id;
    int start_angle;
    int end_angle;
    int center_angle;
    float distance;
    int radial_width;
    float linear_width;
} Object;

Object detected_objects[4]; // Array to hold detected objects
Object smallest_object[1]; // Array holds smallest object for easy access
int object_count = 0;

void scan_field(void);
void clean_data(void);
void object_detect(float distance, int current_angle);
void print_detected_objects(void);

volatile int overflow_count;
volatile int flag;
volatile char uart_data;

float sensor_data_array2_cleaned[91];
float sensor_data_array2[91];

float ir_data_array[91];
float ir_data_array_clean[91];


//void wait_for_calibration(void);
//void turn_to_heading(oi_t *sensor_data, float target_deg);


// I am working on getting the cybot to avoid objects (without the IMU)
int main(void) {
    timer_init();
    adc_init();
    uart_init();
    ping_init();
    servo_init();

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    scan_field();

//    // Initialize I2C1 + BNO055 and switch to 9-DOF sensor fusion
//    init_imu();
//    set_imu_mode(0x0C);
//
//    // Step 1: Calibrate (hold still 2s for Gyro, then spin 360 deg for Mag)
//    wait_for_calibration();
//
//    // Step 2: Rotate to face South
//    turn_to_heading(sensor_data, SOUTH_HEADING);
//
//    // Step 3: Drive forward using your existing movement function
//    double cm_driven = move_forward_until_stop(sensor_data, 250, 150);
//    lcd_printf("Stopped at %.1f cm\nFlag: %d", cm_driven, flag);
//
//    oi_free(sensor_data);
    return 0;
}


// This function minimizes the code to save space
// Both manual and autonomous scan and can't move while scanning
void scan_field() {
    servo_move(0.0); //makes sure scan starts from 0 degrees
    timer_waitMillis(500);

    // does the 180 degree scan

    int i = 0;
    int j = 0;


    for(i; i<=180; i += 2) {
        sensor_data_array2[i/2] = 0;
        ir_data_array[i/2] = 0;
        for(j = 0; j < 3; j++) {
            servo_move(i); //moves servo to correct angle, next 2 lines do actual scanning...
            sensor_data_array2[i/2] += (ping_read()/3.0);
            ir_data_array[i/2] += (adc_read()/3.0);
        }
         timer_waitMillis(500);
    }

    clean_data();

    i = 0;
    for (i = 0; i < 91; i++) {
        object_detect(ir_data_array_clean[i], i*2);
    }
    print_detected_objects();
}

void clean_data(void) {
    int i = 0;
    int object_state = 0; // 0: not on object, 1: on object
    int last_seg = 0;
    float val = 0.0f;

    for (i = 0; i < 91; i++) {
        val = MAX_DIST_CM;
        if (ir_data_array[i] > 0) {
            val = pow((ir_data_array[i] / 18538), -1.225);
            if (val > MAX_DIST_CM || val != val) val = MAX_DIST_CM;
        }
        ir_data_array_clean[i] = val;
    }


    float running_sum = ir_data_array_clean[0];
    int segment_start = 0;
    int count = 1;

    for(i = 1; i < 91; i++) {
        float avg = running_sum / count;
        if(fabs(ir_data_array_clean[i+1] - avg) > DELTA_ERROR) {
            if(fabs(ir_data_array_clean[i+2] - avg) > DELTA_ERROR && i < 90){
                int a;
                for(a = segment_start; a <= i; a++) {
                    ir_data_array_clean[a] = avg;
                }
                segment_start = i + 1;
                running_sum = ir_data_array_clean[segment_start];
                count = 1;
            }
        }else {
            running_sum += ir_data_array_clean[i];
            count++;
        }
    }

    for (i = 0; i < 91; i++) {
        double pulse_ms = (double)sensor_data_array2[i] / 16000.0;
        double distance_cm = (pulse_ms * 34.0) / 2.0;
        sensor_data_array2_cleaned[i] = (float) distance_cm - 4; //subtraction is to account for error
    }

}

// This detects objects and adds them into an array where the data can be used to find the biggest gap (start and end angles)
void object_detect(float distance, int current_angle){
    static int on_object = 0;
    static int start_angle = 0;
    static int prev_angle = 0;
    static float prev_dist = -1.0f; // Initialized to invalid value to indicate no previous distance
    static float dist_sum = 0.0f;
    static int sample_count = 0;
    float current_dist = distance;

    if (current_angle == 0) {
        on_object = 0;
        start_angle = 0;
        prev_angle = 0;
        prev_dist = distance;
        dist_sum = 0.0f;
        sample_count = 0;
    }

    if (!on_object) {//check if we are currently on an object or not
        // Look for a large enough distance drop to indicate the leading edge of an object
        if ((prev_dist - current_dist) > DELTA_THRESHOLD) {
            on_object = 1;
            start_angle = current_angle;
            dist_sum = current_dist;
            sample_count = 1;
        }
    } else {
        // Currently on an object
        // Look for when distance jumps back up significantly
        if ((current_dist - prev_dist) > DELTA_THRESHOLD) {
            on_object = 0;
            int end_angle = prev_angle; // The object ended at the previous angle
            int radial_w = end_angle - start_angle;


            // check if the angle detected is too small to be an object
            if (radial_w >= 4 && object_count < 4) {
                int center_angle = (start_angle + end_angle) / 2;

                //use the sonar for more accurate distance, but use the IR to get the angle and width of the object
                float dist = 0.0;
                dist = (sensor_data_array2_cleaned[center_angle / 2 + 1] + sensor_data_array2_cleaned[center_angle / 2 -1 ]) / 2.0; //average the two closest readings to get a more accurate distance


                detected_objects[object_count].id = object_count + 1; //update all object values in the struct
                detected_objects[object_count].start_angle = start_angle;
                detected_objects[object_count].end_angle = end_angle;
                detected_objects[object_count].center_angle = center_angle;
                detected_objects[object_count].radial_width = radial_w;
                detected_objects[object_count].distance = dist;

                // Calculate the linear width of the detected object
                float theta_rad = (radial_w * M_PI) / 180.0;
                detected_objects[object_count].linear_width = 2.0 * dist * sin(theta_rad / 2.0);

                object_count++;
            }
        } else {
            // Still on the object add all readings
            dist_sum += current_dist;
            sample_count++;
        }
    }

    // Update history for next function call
    prev_dist = current_dist;
    prev_angle = current_angle;

    if(current_angle == 180) {//catches object if it is at the edge of the scan range
        if (on_object) {
            int end_angle = prev_angle; // The object ended at the previous angle
            int radial_w = end_angle - start_angle;

            // check if the angle detected is too small to be an object or max objects detected
            if (radial_w >= 4 && object_count < 4) {

                int center_angle = (start_angle + end_angle) / 2;

                float dist = 0.0;
                dist = (sensor_data_array2_cleaned[center_angle / 2 + 1] + sensor_data_array2_cleaned[center_angle / 2 -1 ]) / 2.0; //average the two closest readings to get a more accurate distance



                detected_objects[object_count].id = object_count + 1; //update all object values in the struct
                detected_objects[object_count].start_angle = start_angle;
                detected_objects[object_count].end_angle = end_angle;
                detected_objects[object_count].center_angle = center_angle;
                detected_objects[object_count].radial_width = radial_w;
                detected_objects[object_count].distance = dist;

                // Calculate the linear width of the detected object
                float theta_rad = (radial_w * M_PI) / 180.0;
                detected_objects[object_count].linear_width = 2.0 * dist * sin(theta_rad / 2.0);

                object_count++;
            }
        }
    }
}

//this function is to be changed into a gap finder, biggest gap takes the W (originally print_detected_objects)
void print_detected_objects(void) {
   char out[80];
   char test[10];
   uart_sendStr("Object#   Angle   Distance   Angular Width   Linear Width\n\r");

   int smallest_idx = -1;
   int min_radial = 181; // Initialize to maximum possible width
   float min_linear = 9999.0f;

   int biggest_gap = 0;
   int left_gap_idx = -1;
   int right_gap_idx = -1;
   int gap_center = 0;

   int i;
   for (i = 0; i < object_count; i++) {
       sprintf(out, "%-7d   %-5d   %-12.1f   %-16d   %-16.1f\n\r",
               detected_objects[i].id, //goes back and prints all of the detected objects to putty
               detected_objects[i].center_angle,
               detected_objects[i].distance,
               detected_objects[i].radial_width,
               detected_objects[i].linear_width);
       uart_sendStr(out);

       if(i+1 < 4) { //this finds the biggest gap based on the amount of degrees between each object, and then finds the center of the gap
           int current_gap = detected_objects[i+1].start_angle - detected_objects[i].end_angle;
           if (current_gap > biggest_gap) {
               biggest_gap = current_gap;
               left_gap_idx = i;
               right_gap_idx = i+1;
               gap_center = detected_objects[right_gap_idx].start_angle - (biggest_gap/2); //this is where the bot should point to/turn to
           }
       }
   }

       servo_move(gap_center); //moves the servo to gap

//       if (detected_objects[i].linear_width < min_linear) { //calculates the smallest linear width and then prints out the object number, angle, and width to putty
//           min_linear = detected_objects[i].linear_width;
//           smallest_idx = i;
//       }
//   }
//
//   if (smallest_idx != -1) {
//       // Print the smallest object's information
//       float center_rad = detected_objects[smallest_idx].center_angle * M_PI / 180.0;
//       int accurate_angle = (int)(atan2(14 + detected_objects[smallest_idx].distance * sin(center_rad),detected_objects[smallest_idx].distance * cos(center_rad)) * (180.0 / M_PI));
//       smallest_object[0] = detected_objects[smallest_idx];
//       smallest_object[0].center_angle = accurate_angle;
//       sprintf(out, "\n\rSmallest length Object: #%d at %d deg (Width: %.2f cm), Accurate Angle: %d deg\n\r",
//               detected_objects[smallest_idx].id,
//               detected_objects[smallest_idx].center_angle,
//               detected_objects[smallest_idx].linear_width,
//               accurate_angle);
//       cyBot_send_string(out);
//       sprintf(test, "%f\n\r", smallest_object[0].distance);
//       cyBot_send_string(test);
//       servo_move(detected_objects[smallest_idx].center_angle); //move the servo to the angle of gap
//   } else {
//       cyBot_send_string("\n\rNo objects detected.\n\r");
//       smallest_object[0].center_angle = 90; //these make sure that the cybot will not move in auto mode if no objects are detected
//       smallest_object[0].distance = 0.0;
//   }
}

//void wait_for_calibration(void){
//    Calibration cal;
//
//
//    while (1) {
//        cal = calibrate_imu();
//        lcd_printf("Calibrating IMU\nG:%d M:%d A:%d S:%d\nSW1 to override",
//                   cal.gyro, cal.magn, cal.accl, cal.syst);
//
//        // Gyro + Mag = 3 gives reliable compass heading on flat ground
//        if (cal.gyro == 3 && cal.magn == 3) {
//            break;
//        }
//
//        // Safety override: press SW1 if indoor metal keeps Mag at 2
//        if (cal.gyro == 3 && button_getButton() == 1) {
//            break;
//        }
//
//        timer_waitMillis(100);
//    }
//
//    lcd_printf("IMU Ready!\nStarting turn...");
//    timer_waitMillis(500);
//}
//
//void turn_to_heading(oi_t *sensor_data, float target_deg) {
//    float heading = 0.0f;
//    float error = 0.0f;
//    int16_t speed = 0;
//
//    while (1) {
//        // Read fused heading directly (0.0 to 360.0 deg, no quaternion.c needed)
//        bno055_convert_float_euler_h_deg(&heading);
//
//        // Compute shortest signed angle error in range [-180, +180]
//        error = target_deg - heading;
//        while (error > 180.0f)  error -= 360.0f;
//        while (error < -180.0f) error += 360.0f;
//
//        lcd_printf("Target: %.1f\nCurr:   %.1f\nErr:    %.1f",
//                   target_deg, heading, error);
//
//        // Stop once inside tolerance band
//        if (fabsf(error) <= HEADING_TOL) {
//            break;
//        }
//
//        // Two-stage speed control: fast turn when far, slow creep near target
//        speed = (fabsf(error) > 15.0f) ? 80 : 35;
//
//        if (error > 0.0f) {
//            // Need heading to increase -> turn clockwise
//            oi_setWheels(-speed, speed);
//        } else {
//            // Need heading to decrease -> turn counterclockwise
//            oi_setWheels(speed, -speed);
//        }
//
//        timer_waitMillis(15); // ~66 Hz update (BNO055 fuses at 100 Hz)
//    }
//
//    oi_setWheels(0, 0);
//    timer_waitMillis(150); // Let chassis settle before driving forward
//}

