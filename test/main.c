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


  printf("moving forward!\n");
  rover_forward(128);
  while(done == false){};
  done = false;
  rover_stop();
  printf("done moving forward\n");

  
  printf("waiting for rover to be done\n");
  while(check_rover_done() == false && done == false){};
  printf("rover is \n");

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
