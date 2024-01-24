#ifndef ROVERCORE_ISR_H
#define ROVERCORE_ISR_H

#include "servo.h"

int isr_init(Servo* rover_servos);

int isr();

#endif
