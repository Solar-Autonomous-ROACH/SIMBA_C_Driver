#include "pid.h"

void PIDController_init(PIDController *pid) {

  /* Clear controller memory */
  pid->integrator = 0.0;
  pid->prevError = 0.0;
  pid->differentiator = 0.0;
  pid->prevMeasurement = 0.0;

  /* Clear output */
  pid->output = 0.0;
}

double PIDController_update(PIDController *pid, double setPoint,
                            double measurement) {

  /* Compute error */
  double error = setPoint - measurement;

  /* Proportional */
  double proportional = pid->Kp * error;

  /* Integral (Euler integration) */
  pid->integrator = pid->integrator + (0.5 * pid->Ki * pid->sampleTime *
                                       (error + pid->prevError));

  /* Integrator anti-wind-up clamping */
  if (pid->integrator > pid->integratorLimitMax) {
    pid->integrator = pid->integratorLimitMax;
  } else if (pid->integrator < pid->integratorLimitMin) {
    pid->integrator = pid->integratorLimitMin;
  }

  /* Derivative (Band-limited differentiator) */
  pid->differentiator =
      -(2.0 * pid->Kd * (measurement - pid->prevMeasurement) +
        (2.0 * pid->tau - pid->sampleTime) * pid->differentiator) /
      (2.0 * pid->tau + pid->sampleTime);

  /* Compute output */
  pid->output = proportional + pid->integrator + pid->differentiator;

  /* Clamp limits */
  if (pid->output > pid->outputLimitMax) {
    pid->output = pid->outputLimitMax;
  } else if (pid->output < pid->outputLimitMin) {
    pid->output = pid->outputLimitMin;
  }

  /* Store states */
  pid->prevError = error;
  pid->prevMeasurement = measurement;

  /* Return output */
  return pid->output;
}
