#include "isr.h"
#include "mmio.h"
#include "motor.h"

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

static bool done = false;

void sigint_handler() {
  printf("Received SIGINT signal\n");

  if (mmio_is_valid()) {
    for (int i = 0; i < 14; i++) {
      set_motor_speed(i, 0);
    }
  }

  done = true;
}

int main() {
  signal(SIGINT, sigint_handler);

  mmio_init();

  isr_init();

  while (!done) {
  }

  close_mem();

  return 0;
}
