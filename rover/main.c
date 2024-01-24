// TEST MMIO GPIO CONTROL
#include "servo.h"
#include "rover.h"

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

  // Close Rover
  if (rover_close() != 0) {
    printf("failed to close rover\n");
    return -1;
  }

  return 0;
}
