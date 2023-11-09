#ifndef PID_H
#define PID_H

// inspired from: https://github.com/pms67/PID/blob/master/PID.h

typedef struct {
  /* Controller gains */
  double Kp;
  double Ki;
  double Kd;

  /* Derivative low-pass filter time constant */
  double tau;

  /* Output limits */
  double outputLimitMin;
  double outputLimitMax;

  /* Integrator limits */
  double integratorLimitMin;
  double integratorLimitMax;

  /* Sample time (seconds) */
  double sampleTime;

  /* Controller memory */
  double integrator;
  double prevError;

  double differentiator;
  double prevMeasurement;

  /* Controller output */
  double output;

} PIDController;

void PIDController_init(PIDController *pid);
double PIDController_update(PIDController *pid, double setPoint,
                            double measurement);

#endif
