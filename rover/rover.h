// Rover Control API
#ifndef ROVER_H
#define ROVER_H

/** include to make accessible in library */
#include "mmio.h"
#include "motor.h"

#include <stdlib.h>
/* Define the Rover's Motors */
#define MOTOR_REAR_RIGHT_STEER (0x80000000)
#define RRS (MOTOR_REAR_RIGHT_STEER)
#define MOTOR_REAR_RIGHT_WHEEL 0x80010000
#define RRW (MOTOR_REAR_RIGHT_WHEEL)

#define MOTOR_FRONT_RIGHT_STEER 0x80020000
#define FRS (MOTOR_FRONT_RIGHT_STEER)
#define MOTOR_FRONT_RIGHT_WHEEL 0x80030000
#define FRW (MOTOR_FRONT_RIGHT_WHEEL)

#define MOTOR_FRONT_LEFT_STEER 0x80040000
#define FLS (MOTOR_FRONT_LEFT_STEER)
#define MOTOR_FRONT_LEFT_WHEEL 0x80050000
#define FLW (MOTOR_FRONT_LEFT_WHEEL)

#define MOTOR_REAR_LEFT_STEER 0x80060000
#define RLS (MOTOR_REAR_LEFT_STEER)
#define MOTOR_REAR_LEFT_WHEEL 0x80070000
#define RLW (MOTOR_REAR_LEFT_WHEEL)

#define MOTOR_MIDDLE_LEFT_WHEEL 0x80080000
#define MLW (MOTOR_MIDDLE_LEFT_WHEEL)
#define MOTOR_MIDDLE_RIGHT_WHEEL 0x80090000
#define MRW (MOTOR_MIDDLE_RIGHT_WHEEL)

#define MOTOR_WRIST 0x800A0000
#define MOTOR_BASE 0x800B0000
#define MOTOR_ELBOW 0x800C0000
#define MOTOR_CLAW 0x800D0000

#define WATCHDOG_REG 0x80100000

#define ROVER_ROTATE_CLOCKWISE 0
#define ROVER_ROTATE_COUNTER 1

/*
#define MOTOR_15 0x800E0000
#define MOTOR_16 0x800F0000
#define MOTOR_17 0x80100000
*/

#define MAX_STEERING_TICKS      300

/* For Motion*/
#define WHEEL_DIAMETER 124     /* mm diameter of the wheel*/
/*this is for the 6 wheel motors*/
#define TICKS_PER_REV_6 12                /*ticks per revolution*/
#define GEAR_RATIO_6 116
/*this is for the 4 rotating motors*/
#define TICKS_PER_REV_4 40                /*ticks per revolution*/
#define GEAR_RATIO_4 172

typedef enum {
    ROVER_CALIBRATE_WAITING,
    ROVER_CALIBRATE_FR,
    ROVER_CALIBRATE_RR,
    ROVER_CALIBRATE_FL,
    ROVER_CALIBRATE_RL,
    ROVER_CALIBRATE_READY
} rover_state_t;

/* API functions */
// Initializes the rover. Return 0 on success, nonzero on failure
int rover_init();
int rover_close();
// calibration
void rover_calibrate();
int rover_is_calibrated();
// Sets the target motor speed and runs it. Return 0 on success, nonzero on
// failur
int motor_set_speed(off_t motor_addr, double speed);
int64_t motor_get_position(off_t motor_addr);
// Rover control. Return 0 on success, nonzero on failure
int rover_move_x(int distance, double speed);
//int rover_rotate(int dir, int angle);
void rover_stop();
void rover_forward(int speed);
void rover_reverse(int speed);

void rover_pointTurn_CW(int speed);
void rover_pointTurn_CCW(int speed);

void rover_steer_forward();
void rover_steer_right(int angle);
void rover_steer_left(int angle);
void rover_steer_point();

// Return a status that if the rover is currently moving or not
int check_rover_done();
/* for isr */
int isr_init();
void rover_isr();
int isr_attach_function(void (*fun)());

#endif
