#include "open_interface.h"
#include "movement.h"
#include "math.h"
#include "Timer.h"
#include "uart.h"


double move_forward(oi_t *sensor, double centimeters){
    int wheel_speed = 150;


    double sum = 0;
    oi_setWheels(wheel_speed, wheel_speed);

    double adj_centimeters = (centimeters*10.0 - 1.5*centimeters ); // - (.4*centimeters);  //- 40.0; //This is the adjustment for the distance traveled, it is based on testing and the amount of centimeters that are wanted to be traveled
    while (sum < adj_centimeters){ //This is the loop that goes until the sum of the distance traveled reaches the amount of wanted centimeters
        oi_update(sensor);
        sum += sensor->distance;

        if(sensor->bumpLeft || sensor->bumpRight){
            break;

        }
    }
    oi_setWheels(0,0);
    return centimeters; //This returns the amount of centimeters traveled, it is divided by 10 to convert from millimeters to centimeters
}

void move_backward(oi_t *sensor, double centimeters){
    int wheel_speed = -150;


    double sum = 0;
    oi_setWheels(wheel_speed, wheel_speed);

    double adj_centimeters = (centimeters*10) - 10.0; //(centimeters*.04); //This is the adjustment for the distance traveled, it is based on testing and the amount of centimeters that are wanted to be traveled
    while (fabs(sum) < adj_centimeters){ //This is the loop that goes until the sum of the distance traveled reaches the amount of wanted centimeters
        oi_update(sensor);
        sum += fabs(sensor->distance);

    }
    oi_setWheels(0,0);
}


void turn_clockwise(oi_t *sensor, int degrees){

    oi_setWheels(-150, 150);


    double sum = 0;

    while(fabs(sum) < (degrees - 2)){ // -12 accounts for the overshoot of the robot when turning, this was found through testing
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0,0);
}

void turn_counter_clockwise(oi_t *sensor, int degrees){

    oi_setWheels(150, -150);

    double sum = 0;

    while(fabs(sum) < (degrees-2)){ // -12 accounts for the overshoot of the robot when turning, this was found through testing
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0,0);
}

void go_around(oi_t *sensor_data, int direction){//direction 0 is move right(left bumper hit), 1 is move left(right bumper hit)
    timer_waitMillis(500);
    move_backward(sensor_data, 15);
    timer_waitMillis(500);

    if(direction == 0){
        turn_clockwise(sensor_data, 90);
        timer_waitMillis(500);
        move_forward(sensor_data, 20);
        timer_waitMillis(500);
        turn_counter_clockwise(sensor_data, 90);
    }else{
        turn_counter_clockwise(sensor_data, 90);
        timer_waitMillis(500);
        move_forward(sensor_data, 20);
        timer_waitMillis(500);
        turn_clockwise(sensor_data, 90);
    }
    timer_waitMillis(500);
}

void uart_sendString(char *string){//this loops through the string to send it
    int i = 0;
    while(string[i] != '\0'){
        uart_sendChar(string[i]);
        i++;
    }
}
