#include "motor.h" // Include the header file "motor.h"

static long motor_abs_pos[MAX_MOTORS];
static long long motor_target_pos[MAX_MOTORS];
static int motor_velocity[MAX_MOTORS];

static uint8_t motor_raw_pos[MAX_MOTORS];
// static uint8_t motor_pwm[MAX_MOTORS];

// This function updates the position of a motor with the given index.
int motor_update(uint8_t motor_index) {
  uint8_t last;   // Variable to store the last position of the motor
  long last_long; // Variable to store the last position of the motor as a long
                  // integer

  // Check if the motor index is out of range
  if (motor_index >= MAX_MOTORS) {
    perror("Motor Index Out Of Range"); // Print an error message
    return -1;                          // Return an error code
  }

  last = motor_raw_pos[motor_index]; // Save the last position of the motor
  last_long = motor_abs_pos[motor_index];
  // Read the current position of the motor from the I/O register
  motor_raw_pos[motor_index] = get_PL_register(MOTOR_READ_REG + motor_index);
  // Check if the motor has crossed a threshold in either direction
  if ((last > HIGH_THRESH) && (motor_raw_pos[motor_index] < LOW_THRESH)) {
    motor_abs_pos[motor_index] +=
        1 << MOTOR_DATA_SIZE; // Increment the absolute position of the motor
  }
  if ((last < LOW_THRESH) && (motor_raw_pos[motor_index] > HIGH_THRESH)) {
    motor_abs_pos[motor_index] -=
        1 << MOTOR_DATA_SIZE; // Decrement the absolute position of the motor
  }

  motor_abs_pos[motor_index] &=
      UPPER_MASK; // Mask the upper bits of the absolute position
  motor_abs_pos[motor_index] +=
      motor_raw_pos[motor_index]; // Add the raw position to the absolute
                                  // position

  motor_velocity[motor_index] = motor_abs_pos[motor_index] - last_long;

  return 0; // Return success code
}

int set_motor_speed(uint8_t motor_index, int speed) {
  if (motor_index >= MAX_MOTORS) {
    perror("Motor Index Out Of Range"); // Print an error message
    return -1;                          // Return an error code
  }
  if (speed > 127)
    speed = 127;
  if (speed < -127)
    speed = -127;
  if (speed >= 0)
    set_PL_register(MOTOR_WRITE_REG + motor_index, speed);
  else
    set_PL_register(MOTOR_WRITE_REG + motor_index, 0xFF + speed);
  return 0;
}

long get_motor_position(uint8_t motor_index) {
  if (motor_index >= MAX_MOTORS) {
    perror("Motor Index Out Of Range");
    return -1;
  }
  return motor_abs_pos[motor_index];
}

char get_raw_pos(uint8_t motor_index) {
  if (motor_index >= MAX_MOTORS) {
    perror("Motor Index Out Of Range");
    return -1;
  }
  return motor_raw_pos[motor_index];
}

long long get_target_position(uint8_t motor_index) {
  if (motor_index >= MAX_MOTORS) {
    perror("Motor Index Out Of Range");
    return -1;
  }
  return motor_target_pos[motor_index];
}

int set_target_position(uint8_t motor_index, long long target_position) {
  if (motor_index >= MAX_MOTORS) {
    perror("Motor Index Out Of Range");
    return -1;
  }
  motor_target_pos[motor_index] = target_position;
  return 0;
}

int get_motor_velocity(uint8_t motor_index) {
  if (motor_index >= MAX_MOTORS) {
    perror("Motor Index Out Of Range");
    return -1;
  }
  return motor_velocity[motor_index];
}
