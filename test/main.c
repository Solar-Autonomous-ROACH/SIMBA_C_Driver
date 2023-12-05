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

  // Setup servo
  Servo servo;
  if (Servo_init(&servo, MOTOR_REAR_LEFT_WHEEL) != 0) {
    printf("failed to initialize servo\n");
    return -1;
  }

  // PID control loop
  double angle = 0;
  while (!done) {
    // Update servo
    Servo_update(&servo);

    // Update the setpoint
    angle += 0.5;
    servo.setpoint = 500.0 * sin(angle * M_PI / 180.0);

    // Delay the loop
    usleep(5000);
  }

  // Close motor controller
  Servo_close(&servo);

  return 0;
}
