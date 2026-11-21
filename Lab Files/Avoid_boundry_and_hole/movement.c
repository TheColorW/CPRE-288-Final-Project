/*
 *   movement.c
 *
 *   Created on: Aug 25, 2026
 *   Updated on: Nov 18, 2026
 *      Authors: carlos28, jfr72506, gm3vf
 */

#include "math.h"
#include "movement.h"


// Move forward at a specified speed
double move_forward(oi_t *sensor, double centimeters, int speed)
{
    double sum = 0;
    oi_setWheels(speed, speed);

    double adj_centimeters = (centimeters * 10.0 - 1.5 * centimeters); // Convert and adjust
    while (sum < adj_centimeters)
    {
        oi_update(sensor); // Update sensor
        sum += sensor->distance; // Add new distance to total distance

        if (sensor->bumpLeft || sensor->bumpRight) // Stop if object is hit
        {
            break;

        }
    }
    oi_setWheels(0, 0); // Stop when complete
    return centimeters; //This returns the amount of centimeters traveled, it is divided by 10 to convert from millimeters to centimeters
}

// Move backward at a specified speed
void move_backward(oi_t *sensor, double centimeters, int speed)
{
    oi_setWheels(speed, speed);
    double sum = 0;
    double adj_centimeters = (centimeters * 10) - 10.0; // Convert

    while (fabs(sum) < adj_centimeters)
    {
        oi_update(sensor);
        sum += fabs(sensor->distance);
    }
    oi_setWheels(0, 0); // Stop when complete
}

// Turn clockwise
void turn_clockwise(oi_t *sensor, int degrees)
{
    oi_setWheels(-150, 150);

    double sum = 0;

    while (fabs(sum) < (degrees - 2)) // -2 accounts for overshoot
    {
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0, 0); // Stop when complete
}

// Turn anticlockwise
void turn_anticlockwise(oi_t *sensor, int degrees)
{
    oi_setWheels(150, -150);

    double sum = 0;

    while (fabs(sum) < (degrees - 2)) // -2 accounts for overshoot
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

void go_around(oi_t *sensor_data, int direction) //direction 0 is move right(left bumper hit), 1 is move left(right bumper hit)
{
    timer_waitMillis(500);
    move_backward(sensor_data, 15, 150);
    timer_waitMillis(500);

    if (direction == 0)
    {
        turn_clockwise(sensor_data, 90);
        timer_waitMillis(500);
        move_forward(sensor_data, 20, 150);
        timer_waitMillis(500);
        turn_anticlockwise(sensor_data, 90);
    }
    else
    {
        turn_anticlockwise(sensor_data, 90);
        timer_waitMillis(500);
        move_forward(sensor_data, 20, 150);
        timer_waitMillis(500);
        turn_clockwise(sensor_data, 90);
    }
    timer_waitMillis(500);
}


//function to which stops if interrupted by a bump or a different surface

volatile int type_flag = 0;



double move_forward_until_stop(oi_t *sensor, double centimeters, int speed){
    double sum = 0;
       oi_setWheels(speed, speed);

       double adj_centimeters = (centimeters * 10.0 - 1.5 * centimeters); // Convert and adjust
       while (sum < adj_centimeters)
       {
           oi_update(sensor); // Update sensor
           sum += sensor->distance; // Add new distance to total distance
           int leftColorSensor = sensor->cliffLeftSignal;
           int frontLeftColorSensor = sensor->cliffFrontLeftSignal;
           int rightColorSensor = sensor->cliffRightSignal;
           int frontRightColorSensor = sensor->cliffFrontRightSignal;

           if (sensor->bumpLeft || sensor->bumpRight) // Stop if object is hit
           {
               flag = 1; //bumper got hit
               break;
           }else if (leftColorSensor > 2650){
               flag = 2; //left wall got hit
               break;
           }else if(rightColorSensor > 2650){
               flag = 3; //right wall got hit
               break;
           }else if(frontLeftColorSensor > 2650 || frontRightColorSensor > 2650){
               flag = 4;//front wall got hit
               break;
           }else if (leftColorSensor > 0 && leftColorSensor < 100){
               flag = 5; //left hole
               break;
           }else if(rightColorSensor > 0 && rightColorSensor < 100){
               flag = 6; //right hole
               break;
           }else if(frontLeftColorSensor > 0 && frontLeftColorSensor < 100 || frontRightColorSensor > 0 && frontRightColorSensor < 100){
               flag = 7; //front hole
               break;
           } //need to make something for if we get the correct final destination

       }
       oi_setWheels(0, 0); // Stop when complete or broken
       return centimeters; //This returns the amount of centimeters traveled, it is divided by 10 to convert from millimeters to centimeters PROBABLY CHANGE TO RETURN SUM NOT CENTIMETERS

}
