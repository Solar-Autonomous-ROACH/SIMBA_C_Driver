#include <aio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "motor.h"
#include "pid.h"
#include "servo.h"

int Servo_init(Servo *servo, off_t mmio_address) {
  // Initialize motor controller
  if (MotorController_init(&(servo->motor), mmio_address) != 0) {
    return -1;
  }

  // Initialize PID controller
  (servo->pid).Kp = 15.0;
  (servo->pid).Ki = 30.0;
  (servo->pid).Kd = 2.0;
  (servo->pid).tau = 0.04;
  (servo->pid).outputLimitMin = -255.0;
  (servo->pid).outputLimitMax = 255.0;
  (servo->pid).integratorLimitMin = -20.0;
  (servo->pid).integratorLimitMax = 20.0;
  (servo->pid).sampleTime = 0.005;
  PIDController_init(&(servo->pid));

  // Initialize control variables
  servo->setpoint = 0;
  servo->counts = 0;
  servo->prevCounts = 0;
  servo->inverted = false;

  return 0;
}

void Servo_close(Servo *servo) {
  // Close motor controller
  MotorController_close(&(servo->motor));
}

void Servo_update(Servo *servo) {
  // Update counts
  MotorController_read(&(servo->motor));
  servo->counts += (servo->motor).counts - servo->prevCounts;
  servo->prevCounts = (servo->motor).counts;

  // Compute control signal
  PIDController_update(&(servo->pid), servo->setpoint, (double)servo->counts);

  // Apply control signal
  (servo->motor).dir = (servo->pid).output >= 0 ? 0 : 1;
  if (servo->inverted) {
    (servo->motor).dir = !(servo->motor).dir;
  }
  (servo->motor).duty_cycle = (uint8_t)fabs((servo->pid).output);
  MotorController_write(&(servo->motor));
}
