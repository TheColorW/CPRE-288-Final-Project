/*
Used to quickly grab CyBot Servo left/right values

1, 3, 4, 6, 8, 11, 20
*/

#ifdef SERVO_CAL_H
#define SERVO_CAL_H

int cal_left[25] = {
    1230250, // Cybot 1
    null, // Cybot 2
    1219750, // Cybot 3
    1230250, // Cybot 4
    null, // Cybot 5
    1272250, //Cybot 6 (THE BEST ONE)
    null, // Cybot 7
    1267000, // Cybot 8
    null, // Cybot 9
    null, // Cybot 10
    1282750, // Cybot 11
    null, // Cybot 12
    null, // Cybot 13
    null, // Cybot 14
    null, // Cybot 15
    null, // Cybot 16
    null, // Cybot 17
    null, // Cybot 18
    null, // Cybot 19
    1240750, // Cybot 20
    null, // Cybot 21
    null, // Cybot 22
    null, // Cybot 23
}

int cal_right[25] = {
    243250, // Cybot 1
    null, // Cybot 2
    274750, // Cybot 3
    285250, // Cybot 4
    null, // Cybot 5
    280000, //Cybot 6 (THE BEST ONE)
    null, // Cybot 7
    311500, // Cybot 8
    null, // Cybot 9
    null, // Cybot 10
    285250, // Cybot 11
    null, // Cybot 12
    null, // Cybot 13
    null, // Cybot 14
    null, // Cybot 15
    null, // Cybot 16
    null, // Cybot 17
    null, // Cybot 18
    null, // Cybot 19
    238000, // Cybot 20
    null, // Cybot 21
    null, // Cybot 22
    null, // Cybot 23
}

#endif