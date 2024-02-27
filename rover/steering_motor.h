#ifndef C_STEERING_MOTOR_H
#define C_STEERING_MOTOR_H

#include <stdint.h>
#include <stdio.h>
#include "servo.h"

#define CALIBRATION_SPEED 3000//was 30

 typedef enum {
    STATE_INITIALIZE,
    STATE_WAITING,
    STATE_CALIBRATION_LEFT,
    STATE_CALIBRATION_RIGHT,
    STATE_CALIBRATION_CENTER,
    STATE_READY
} state_t;

typedef struct {
    // legacy
    uint8_t index;
    // replaces index
    Servo* servo;
    long left_pos;
    long right_pos;
    long center_pos;
    int target;
    state_t state;  // current state of the motor
} steering_motor_t;


void calibrate (steering_motor_t *s_motor);
int steering_motor_handle_state(steering_motor_t *s_motor);

#endif //C_STEERING_MOTOR_H
