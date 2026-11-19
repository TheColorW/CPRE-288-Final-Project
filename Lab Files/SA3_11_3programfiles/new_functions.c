#include "open_interface.h"
#include "Timer.h"
#include "movement.h"
#include "lcd.h"


void main(){
    lcd_init();
    timer_init();
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    //double tot_distance_left = (200.0-25.0); //This is the total distance the robot as left to go

    while(1){ //will keep running until reach the distance wanted
//        double distance_traveled = move_forward(sensor_data, tot_distance_left);
//        tot_distance_left -= distance_traveled; //configure how much is left to traverse

//        if(sensor_data->bumpLeft && sensor_data->bumpRight){ //if the robot bumps into something, it will go around it
//            go_around(sensor_data, 0); //go around the object in front of it
//            tot_distance_left += 15;
//        }else if(sensor_data->bumpLeft){ //if the robot bumps into something on the left, it will go around it
//            go_around(sensor_data, 0); //go around the object on the right
//            tot_distance_left +=15;
//        }else if(sensor_data->bumpRight){ //if the robot bumps into something on the right, it will go around it
//            go_around(sensor_data, 1); //go around the object on the left
//            tot_distance_left +=15;
//        }

        //50 was black, 2500 was grey, and 2800 was white... rough estimates based on returned values
        lcd_printf("%d", sensor_data->cliffLeftSignal);
        timer_waitMillis(3000);

        // Twinkle Twinkle little star
        //unsigned char note[14] = {60, 60, 67, 67, 69, 69, 31, 65, 65, 64, 64, 62, 62, 60};
        //unsigned char duration[14] = {32, 32, 32, 32, 32, 32, 64, 32, 32, 32, 32, 32, 32, 64};

        //oi_loadSong(1, 14, &note, &duration);
        //oi_play_song(1);
        //timer_waitMillis(3000);

//        if(sensor_data->cliffLeft){
//            turn_clockwise(sensor_data, 90);
//        }
    }

    
    //oi_setWheels(0,0);
    //oi_free(sensor_data);
}
