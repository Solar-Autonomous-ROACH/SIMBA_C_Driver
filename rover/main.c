// TEST MMIO GPIO CONTROL
#include "rover.h"
#include "servo.h"
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

int main() {
  // Configure signal handler
  signal(SIGINT, sigint_handler);

  // Initialize rover
  if (rover_init() != 0) {
    printf("failed to initialize rover\n");
    return -1;
  }

  // infinite loop
  while (done == false) {
  }

  // Close Rover
  if (rover_close() != 0) {
    printf("failed to close rover\n");
    return -1;
  }

  // testing
  // motor_set_speed(0, 0);
  return 0;
}
