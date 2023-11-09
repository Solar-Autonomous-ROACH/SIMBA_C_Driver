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
  volatile unsigned int *mmio = mmio_init(IOMEM_ADDRESS);

  if (!mmio_is_valid(mmio)) {
    printf("mmio is invalid\n");
    return -1;
  } else {
    printf("mmio is valid\n");
  }

  uint8_t duty_cycle = 0;  // 8 bits
  uint8_t clk_divisor = 4; // 3 bits
  uint8_t dir = 1;         // 1 bit
  uint8_t en_motor = 1;    // 1 bit
  uint8_t clear_enc = 1;   // 1 bit
  uint8_t en_enc = 1;      // 1 bit

  *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en_motor << 12) +
          (clear_enc << 13) + (en_enc << 14);

  clear_enc = 0;

  while (!done) {
    for (duty_cycle = 0; duty_cycle < 255 && !done; duty_cycle++) {
      usleep(50000);
      *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en_motor << 12) +
              (clear_enc << 13) + (en_enc << 14);
      printf("enc: %d\n", *(mmio + 2));
    }
    for (duty_cycle = 255; duty_cycle > 0 && !done; duty_cycle--) {
      usleep(50000);
      *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en_motor << 12) +
              (clear_enc << 13) + (en_enc << 14);
      printf("enc: %d\n", *(mmio + 2));
    }

    dir = !dir;
  }

  en_motor = 0;
  en_enc = 0;

  *mmio = duty_cycle + (clk_divisor << 8) + (dir << 11) + (en_motor << 12) +
          (clear_enc << 13) + (en_enc << 14);

  printf("closing mmio\n");
  close_mem(mmio);

  return 0;
}
