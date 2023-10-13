#include "led.h"

void set_brightness(uint8_t r, uint8_t g, uint8_t b) {
  set_PL_register(LED_R_REG, r);
  set_PL_register(LED_B_REG, b);
  set_PL_register(LED_G_REG, g);
}
void set_led_status() { set_PL_register(LED_MODE_REG, 0x00); }
