// TEST MMIO GPIO CONTROL
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
  if (isr_init() != 0) {
    printf("failed to initialize rover\n");
    return -1;
  }
  
  printf("calibrating...\n");
  // wait for calibration to finish
  while (rover_is_calibrated() == false && done == false){}
  printf("done.\n");

  // testing
  rover_steer_left(300);
  while (check_rover_done() == 0 && done == false) {
  };
  printf("done\n");

  rover_move_x(100, 1000);
  while (check_rover_done() == 0 && done == false) {
  };
  printf("done\n");
 
  rover_steer_forward();
  while (check_rover_done() == 0 && done == false) {
  };
  printf("done\n");

  // infinite loop
  while (done == false) {
  }

  // Close Rover
  if (rover_close() != 0) {
    printf("failed to close rover\n");
    return -1;
  }

  return 0;
}
