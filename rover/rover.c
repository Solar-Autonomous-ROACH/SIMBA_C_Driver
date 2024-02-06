// Rover Control API
#include "rover.h"
#include "isr.h"
#include "mmio.h"
#include "servo.h"

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#define NUM_MOTORS 15
Servo servos[NUM_MOTORS];
double servo_speeds[NUM_MOTORS];
volatile unsigned int *watchdog_flag;

double angle = 0;
double setpoint = 0;
double tiny_setpoint = 0;

/* API functions */
// Sets the target motor speed. Return 0 on success, nonzero on failure
int motor_set_speed(off_t motor_addr, int speed) {
  // find the servo that is associated with the motor_addr
  printf("0x%lx, %d\n", motor_addr, speed);
  for (int i = 0; i < NUM_MOTORS; i++) {
    printf("0x%lx\n", servos[i].motor.addr);
    if (motor_addr == servos[i].motor.addr) {
      printf("found servo[%d] = %lx\n", i, servos[i].motor.addr);
      return 0;
    }
  }
  return -1;
}

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

  // Handle watchdog
  watchdog_flag = mmio_init((off_t)WATCHDOG_REG);

  return 0;
}

// PID Control
int isr() {
  // Handle watchdog
  *(watchdog_flag) = *(watchdog_flag) ? 0 : 1;

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
    // setting servo speeds to zero
    servo_speeds[i] = 0;
    // initializing servos
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
