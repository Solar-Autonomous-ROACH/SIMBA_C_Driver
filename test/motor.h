#ifndef MOTOR_H
#define MOTOR_H

typedef struct {

  // MMIO Address
  off_t mmio_address;

  // GPIO Output Control Signals
  uint8_t duty_cycle;  // 8 bits
  uint8_t clk_divisor; // 3 bits
  uint8_t dir;         // 1 bit
  uint8_t en_motor;    // 1 bit
  uint8_t clear_enc;   // 1 bit
  uint8_t en_enc;      // 1 bit

  // GPIO Input Control Signals
  int16_t counts; // 16 bits

} MotorController;

int MotorController_init(MotorController *motor, off_t mmio_address);
void MotorController_close(MotorController *motor);

void MotorController_write(MotorController *motor);
void MotorController_read(MotorController *motor);

#endif
