#include "isr.h"

// TODO: move to different file
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
  timer.it_interval.tv_usec = ISR_DELAY;
  timer.it_value.tv_sec = 0;
  timer.it_value.tv_usec = ISR_DELAY;
  setitimer(ITIMER_REAL, &timer, NULL);

  // Handle watchdog
  watchdog_flag = mmio_init((off_t)WATCHDOG_REG);

  return 0;
}

// PID Control
// TODO: move to different file
void isr(int signum __attribute__((unused))) {
  // Handle watchdog
  *(watchdog_flag) = *(watchdog_flag) ? 0 : 1;

  // handle calibration, only one motor for now
  for (unsigned i = 0; i < isr_num_functions; i++) {
    isr_functions[i]();
  }
}
