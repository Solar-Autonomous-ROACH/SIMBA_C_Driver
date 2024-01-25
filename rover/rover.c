// Rover Control API
#include "rover.h"
#include "isr.h"
#include "servo.h"

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#define NUM_MOTORS 15
Servo servos[NUM_MOTORS];

double angle = 0;
double setpoint = 0;
double tiny_setpoint = 0;

int isr_init() {
  struct sigaction sa;
  struct itimerval timer;

  // Install the ISR
  sa.sa_handler = (void *)isr;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGALRM, &sa, NULL);

  // Set the timer to trigger every 1ms
  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_usec = 5000; // was 1000
  timer.it_value.tv_sec = 0;
  timer.it_value.tv_usec = 5000; // was 1000
  setitimer(ITIMER_REAL, &timer, NULL);

  // // Stop all motors
  // for (int i = 0; i < 14; i++) {
  //   set_motor_speed(i, 0);
  // }

  return 0;
}

// PID Control
int isr() {
  // TODO
  // Handle watchdog
  // static uint8_t watchdog_flag = 0;
  // set_PL_register(WATCHDOG_REG, watchdog_flag);
  // watchdog_flag = !watchdog_flag;

  // Update servos
  for (int i = 0; i < 10; i++) {
    Servo_update(&servos[i]);
  }

  // Update the setpoint
  angle += 0.5;
  setpoint = 500.0 * sin(angle * M_PI / 180.0);
  tiny_setpoint = 250.0 * sin(angle * M_PI / 180.0);

  servos[0].setpoint = setpoint;
  servos[1].setpoint = -setpoint;
  servos[2].setpoint = setpoint;
  servos[3].setpoint = -setpoint;
  servos[4].setpoint = setpoint;
  servos[5].setpoint = -setpoint;
  servos[6].setpoint = tiny_setpoint;
  servos[7].setpoint = tiny_setpoint;
  servos[8].setpoint = tiny_setpoint;
  servos[9].setpoint = tiny_setpoint;

  return 0;
}

/**
 * @brief the rover.
 * @return 0 on success, nonzero on failure
 */
int rover_init() {
  // define servos and motors
  off_t motor_addrs[NUM_MOTORS] = {
      MOTOR_REAR_LEFT_WHEEL,   MOTOR_REAR_RIGHT_WHEEL,

      MOTOR_MIDDLE_LEFT_WHEEL, MOTOR_MIDDLE_RIGHT_WHEEL,

      MOTOR_FRONT_LEFT_WHEEL,  MOTOR_FRONT_RIGHT_WHEEL,

      MOTOR_FRONT_LEFT_STEER,  MOTOR_FRONT_RIGHT_STEER,

      MOTOR_REAR_LEFT_STEER,   MOTOR_REAR_RIGHT_STEER,

      // MOTOR_WRIST,
      // MOTOR_BASE,
      // MOTOR_ELBOW,
      // MOTOR_CLAW
      //... more unused motors
  };

  // setup servos
  for (int i = 0; i < 10; i++) {
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
  for (int i = 0; i < 10; i++) {
    Servo_close(&servos[i]);
  }
  return 0;
}
