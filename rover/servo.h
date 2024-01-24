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
} Servo;

int Servo_init(Servo *servo, off_t mmio_address);
void Servo_close(Servo *servo);

void Servo_update(Servo *servo);

#define NUM_MOTORS 15
Servo servos[NUM_MOTORS];

#endif
