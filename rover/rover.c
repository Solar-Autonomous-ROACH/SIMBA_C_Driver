// Rover Control API
#include "rover.h"
#include "servo.h"
#include "isr.h"

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

static bool done = false;

void sigint_handler() {
  printf("SIGINT caught\n");
  done = true;
}

/**
 * @brief the rover.
 * @return 0 on success, nonzero on failure
*/
int rover_init() {
    // Configure signal handler
   signal(SIGINT, sigint_handler);

    // define servos and motors
    int motor_addrs[NUM_MOTORS] = {
        MOTOR_REAR_RIGHT_WHEEL,
        MOTOR_REAR_LEFT_WHEEL,
        MOTOR_MIDDLE_RIGHT_WHEEL,
        MOTOR_FRONT_RIGHT_WHEEL,
        MOTOR_FRONT_LEFT_WHEEL,
        MOTOR_FRONT_RIGHT_STEER,
        MOTOR_FRONT_LEFT_STEER,
        MOTOR_REAR_LEFT_STEER,
        MOTOR_REAR_RIGHT_STEER,
        MOTOR_MIDDLE_LEFT_WHEEL,
        MOTOR_MIDDLE_RIGHT_WHEEL,
        MOTOR_WRIST,
        MOTOR_BASE,
        MOTOR_ELBOW,
        MOTOR_CLAW
        //... more unused motors
    };

    // setup servos
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (Servo_init(&servos[i], motor_addrs[i]) != 0) {
            printf("failed to initialize servo%d\n", i);
            return -1;
        }
    }

    // setup isr
    if (isr_init() != 0) {
        printf("failed to initialize isr\n");
        return -1;
    }
    return 0;
}

/**
 * @brief Closes out all the motor connections
 * @return 0 on success, nonzero on failure
 */
int rover_close() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        Servo_close(&servos[i]);
    }
    return 0;
}