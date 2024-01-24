#include "isr.h"
#include "mmio.h"
#include "servo.h"
#include "rover.h"

#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

Servo servos[NUM_MOTORS];

int isr_init(Servo* rover_servos) {
  struct sigaction sa;
  struct itimerval timer;

  // Initialize servos
  for (int i = 0; i < NUM_MOTORS; i++) {
    servos[i] = rover_servos[i];
  }

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

/* PID Control */
int isr() {
  // TODO
  // Handle watchdog
  // static uint8_t watchdog_flag = 0;
  // set_PL_register(WATCHDOG_REG, watchdog_flag);
  // watchdog_flag = !watchdog_flag;

  // Update servos
  for (int i = 0; i < 9; i++) {
    Servo_update(&servos[i]);
  }

  // Update the setpoint
  double angle = 0;
  double setpoint = 0;
  double tiny_setpoint = 0;
  angle += 0.5;
  setpoint = 500.0 * sin(angle * M_PI / 180.0);
  tiny_setpoint = 250.0 * sin(angle * M_PI / 180.0);

  servos[0].setpoint = setpoint;
  servos[1].setpoint = -setpoint;
  servos[2].setpoint = setpoint;
  servos[3].setpoint = -setpoint;
  servos[4].setpoint = setpoint;
  servos[5].setpoint = tiny_setpoint;
  servos[6].setpoint = tiny_setpoint;
  servos[7].setpoint = tiny_setpoint;
  servos[8].setpoint = tiny_setpoint;

  return 0;
}
