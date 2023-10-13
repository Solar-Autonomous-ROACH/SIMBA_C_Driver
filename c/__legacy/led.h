#ifndef LED_H
#define LED_H

#include "mmio.h"
#include <stdint.h>

#define LED_R_REG 0x10
#define LED_G_REG 0x11
#define LED_B_REG 0x12

#define LED_MODE_REG 0x13

void set_brightness(uint8_t r, uint8_t g, uint8_t b);
void set_led_status();

#endif
