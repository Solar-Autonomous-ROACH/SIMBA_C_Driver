// Rover Control API
#ifndef ROVER_H
#define ROVER_H

#include <stdlib.h>

/* Define the Rover's Motors */
#define MOTOR_REAR_RIGHT_STEER 0x80000000
#define MOTOR_REAR_RIGHT_WHEEL 0x80010000

#define MOTOR_FRONT_RIGHT_STEER 0x80020000
#define MOTOR_FRONT_RIGHT_WHEEL 0x80030000

#define MOTOR_FRONT_LEFT_STEER 0x80040000
#define MOTOR_FRONT_LEFT_WHEEL 0x80050000

#define MOTOR_REAR_LEFT_STEER 0x80060000
#define MOTOR_REAR_LEFT_WHEEL 0x80070000

#define MOTOR_MIDDLE_LEFT_WHEEL 0x80080000
#define MOTOR_MIDDLE_RIGHT_WHEEL 0x80090000

#define MOTOR_WRIST 0x800A0000
#define MOTOR_BASE 0x800B0000
#define MOTOR_ELBOW 0x800C0000
#define MOTOR_CLAW 0x800D0000

#define WATCHDOG_REG 0x80100000

/*
#define MOTOR_15 0x800E0000
#define MOTOR_16 0x800F0000
#define MOTOR_17 0x80100000
*/

/* API functions */
// Initializes the rover. Return 0 on success, nonzero on failure
int rover_init();
int rover_close();
// Calibrate the rover. Return 0 on success, nonzero on failure
int rover_calibrate(off_t motor_addr);
// Sets the target motor speed and runs it. Return 0 on success, nonzero on
// failure
int motor_set_speed(off_t motor_addr, int64_t speed);
// Rover control. Return 0 on success, nonzero on failure
int rover_move_x(int64_t distance, double speed);
int rover_rotate(int dir, int angle);
// Return a status that if the rover is currently moving or not
int check_rover_done();
/* for isr */
int isr_init();
int isr();

#endif
