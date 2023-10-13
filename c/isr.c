#include "isr.h"
#include "mmio.h"
#include "motor.h"

#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

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
  timer.it_interval.tv_usec = 1000; // was 1000
  timer.it_value.tv_sec = 0;
  timer.it_value.tv_usec = 1000; // was 1000
  setitimer(ITIMER_REAL, &timer, NULL);

  // Stop all motors
  for (int i = 0; i < 14; i++) {
    set_motor_speed(i, 0);
  }

  return 0;
}

int isr() {
  static uint8_t watchdog_flag = 0;

  // Handle watchdog
  set_PL_register(WATCHDOG_REG, watchdog_flag);
  watchdog_flag = !watchdog_flag;

  // Handle motors
  for (int i = 0; i < 14; i++) {
    motor_update(i);
  }

  // Do something...
  printf("M1_POS: %ld\n", get_motor_position(0));

  return 0;
}
