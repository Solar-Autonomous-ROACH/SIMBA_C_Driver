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
  Servo servo0;
  if (Servo_init(&servo0, MOTOR_REAR_LEFT_WHEEL) != 0) {
    printf("failed to initialize servo0\n");
    return -1;
  }

  Servo servo1;
  if (Servo_init(&servo1, MOTOR_REAR_RIGHT_WHEEL) != 0) {
    printf("failed to initialize servo1\n");
    return -1;
  }

  Servo servo2;
  if (Servo_init(&servo2, MOTOR_MIDDLE_RIGHT_WHEEL) != 0) {
    printf("failed to initialize servo2\n");
    return -1;
  }

  Servo servo3;
  if (Servo_init(&servo3, MOTOR_FRONT_RIGHT_WHEEL) != 0) {
    printf("failed to initialize servo3\n");
    return -1;
  }

  Servo servo4;
  if (Servo_init(&servo4, MOTOR_FRONT_LEFT_WHEEL) != 0) {
    printf("failed to initialize servo4\n");
    return -1;
  }

  // PID control loop
  double angle = 0;
  double setpoint = 0;
  while (!done) {
    // Update servo
    Servo_update(&servo0);
    Servo_update(&servo1);
    Servo_update(&servo2);
    Servo_update(&servo3);
    Servo_update(&servo4);

    // Update the setpoint
    angle += 0.5;
    setpoint = 500.0 * sin(angle * M_PI / 180.0);

    servo0.setpoint = setpoint;
    servo1.setpoint = setpoint;
    servo2.setpoint = setpoint;
    servo3.setpoint = setpoint;
    servo4.setpoint = setpoint;

    // Delay the loop
    usleep(5000);
  }

  // Close motor controller
  Servo_close(&servo0);
  Servo_close(&servo1);
  Servo_close(&servo2);
  Servo_close(&servo3);
  Servo_close(&servo4);

  return 0;
}
