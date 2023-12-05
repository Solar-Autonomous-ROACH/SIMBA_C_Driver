// TEST MMIO GPIO CONTROL

#include "motor.h"
#include "pid.h"
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

  // Setup motor controller
  MotorController motor;
  if (MotorController_init(&motor, MOTOR_REAR_LEFT_WHEEL) != 0) {
    printf("failed to initialize motor controller\n");
    return -1;
  }

  // Initialize PID controller
  PIDController pid;
  pid.Kp = 15.0;
  pid.Ki = 30.0;
  pid.Kd = 2.0;
  pid.tau = 0.04;
  pid.outputLimitMin = -255.0;
  pid.outputLimitMax = 255.0;
  pid.integratorLimitMin = -20.0;
  pid.integratorLimitMax = 20.0;
  pid.sampleTime = 0.005;

  PIDController_init(&pid);

  double setpoint = 0.0;
  double angle = 0;

  // PID control loop
  int64_t counts = 0;
  int16_t prevCount = 0;
  while (!done) {
    // Read the encoder
    MotorController_read(&motor);
    counts += motor.counts - prevCount;
    prevCount = motor.counts;

    // Compute control signal
    PIDController_update(&pid, setpoint, (double)counts);

    // Apply control signal
    motor.dir = pid.output >= 0 ? 0 : 1;
    motor.duty_cycle = (uint8_t)fabs(pid.output);
    MotorController_write(&motor);

    printf("PID: meas %f, out %f, dir %d, duty %d, set %f\n", (double)counts,
           pid.output, motor.dir, motor.duty_cycle, setpoint);

    // Update the setpoint
    angle += 0.5;
    setpoint = 500.0 * sin(angle * M_PI / 180.0);

    // Delay the loop
    usleep(5000);
  }

  // Close motor controller
  MotorController_close(&motor);

  return 0;
}
