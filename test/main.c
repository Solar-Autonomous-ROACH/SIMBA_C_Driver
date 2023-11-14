// TEST MMIO GPIO CONTROL

#include "mmio.h"
#include "pid.h"

#include <math.h>
#include <signal.h>
#include <stdbool.h>
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

  // Initialize mmio
  printf("initializing mmio\n");
  volatile unsigned int *mmio = mmio_init(IOMEM_ADDRESS);

  if (!mmio_is_valid(mmio)) {
    printf("mmio is invalid\n");
    return -1;
  } else {
    printf("mmio is valid\n");
  }

  // Initialize motor controller
  uint8_t duty_cycle = 0;  // 8 bits
  uint8_t clk_divisor = 4; // 3 bits
  uint8_t dir = 1;         // 1 bit
  uint8_t en_motor = 1;    // 1 bit
  uint8_t clear_enc = 1;   // 1 bit
  uint8_t en_enc = 1;      // 1 bit

  *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en_motor << 12) +
          (clear_enc << 13) + (en_enc << 14);

  clear_enc = 0;

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
  int16_t prevCount;
  int16_t currCount;
  while (!done) {
    // Read the encoder
    currCount = *(mmio + 2);
    counts += currCount - prevCount;
    prevCount = currCount;

    // Compute control signal
    PIDController_update(&pid, setpoint, (double)counts);

    // Apply control signal
    dir = pid.output >= 0 ? 1 : 0;
    duty_cycle = (uint8_t)fabs(pid.output);
    *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en_motor << 12) +
            (clear_enc << 13) + (en_enc << 14);

    printf("PID: meas %f, out %f, dir %d, duty %d, set %f\n", (double)counts,
           pid.output, dir, duty_cycle, setpoint);

    // Update the setpoint
    angle += 0.5;
    setpoint = 500.0 * sin(angle * M_PI / 180.0);

    // Delay the loop
    usleep(5000);
  }

  // Close motor controller
  en_motor = 0;
  en_enc = 0;

  *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en_motor << 12) +
          (clear_enc << 13) + (en_enc << 14);

  // Close mmio
  printf("closing mmio\n");
  close_mem(mmio);

  return 0;
}
