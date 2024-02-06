#ifndef SERVO_H
#define SERVO_H

#include <aio.h>
#include <stdbool.h>
#include <stdint.h>

#include "motor.h"
#include "pid.h"

typedef struct {
  // Motor
  MotorController motor;

  // PID
  PIDController pid;

  // Control Variables
  double setpoint;
  int64_t counts;
  int16_t prevCounts;
  bool inverted;
  int64_t speed;
} Servo;

int Servo_init(Servo *servo, off_t mmio_address, bool inverted);
void Servo_close(Servo *servo);

void Servo_update(Servo *servo);

#endif
