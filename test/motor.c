#include <aio.h>

#include "mmio.h"
#include "motor.h"

int MotorController_init(MotorController *motor, off_t mmio_address) {
  // Initialize MMIO
  volatile unsigned int *mmio = mmio_init(mmio_address);

  if (!mmio_is_valid(mmio))
    return -1;

  // Initialize motor controller
  motor->mmio = mmio;
  motor->duty_cycle = 0;
  motor->clk_divisor = 4;
  motor->dir = 1;
  motor->en_motor = 1;
  motor->clear_enc = 1;
  motor->en_enc = 1;

  // Clear encoder counter
  MotorController_write(motor);
  motor->clear_enc = 0;
  MotorController_write(motor);

  return 0;
}

void MotorController_close(MotorController *motor) {
  // Disable motor controller
  motor->en_motor = 0;
  motor->en_enc = 0;
  MotorController_write(motor);

  // Release MMIO
  close_mem(motor->mmio);
}

void MotorController_write(MotorController *motor) {
  // Write to MMIO
  *(motor->mmio) = motor->duty_cycle + (motor->clk_divisor << 8) +
                   (motor->dir << 11) + (motor->en_motor << 12) +
                   (motor->clear_enc << 13) + (motor->en_enc << 14);
}

void MotorController_read(MotorController *motor) {
  // Read MMIO
  motor->counts = *(motor->mmio + 2);
}
