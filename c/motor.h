#ifndef ROVERCORE_MOTOR_H
#define ROVERCORE_MOTOR_H

#include "mmio.h"
#include <stdint.h>

#define MOTOR_READ_REG 0x30
#define MOTOR_WRITE_REG 0x20

#define MAX_MOTORS 16
#define MOTOR_DATA_SIZE 8

#define UPPER_MASK 0xFFFFFF00
#define LOWER_MASK 0x000000FF

#define HIGH_THRESH 206 // 256-50
#define LOW_THRESH 50

#define KP 2
#define KV 2

char get_raw_pos(uint8_t motor_index);

int motor_update(uint8_t motor_index);

int set_motor_speed(uint8_t motor_index, int speed);

long get_motor_position(uint8_t motor_index);

long long get_target_position(uint8_t motor_index);

int set_target_position(uint8_t motor_index, long long target_position);

int get_motor_velocity(uint8_t motor_index);

#endif // ROVERCORE_MOTOR_H
