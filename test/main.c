// TEST MMIO GPIO CONTROL

#include "mmio.h"

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
  signal(SIGINT, sigint_handler);

  printf("initializing mmio\n");
  volatile unsigned int *mmio = mmio_init();

  if (!mmio_is_valid()) {
    printf("mmio is invalid\n");
    return -1;
  } else {
    printf("mmio is valid\n");
  }

  uint8_t duty_cycle = 0;  // 8 bits
  uint8_t clk_divisor = 4; // 3 bits
  uint8_t dir = 1;         // 1 bit
  uint8_t en = 1;          // 1 bit

  *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en << 12);

  while (!done) {
    for (duty_cycle = 0; duty_cycle < 255 && !done; duty_cycle++) {
      usleep(50000);
      *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en << 12);
    }
    for (duty_cycle = 255; duty_cycle > 0 && !done; duty_cycle--) {
      usleep(50000);
      *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en << 12);
    }
  }

  en = 0;

  *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en << 12);

  printf("closing mmio\n");
  close_mem();

  return 0;
}
