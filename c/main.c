//
// Created by Tyler Bovenzi on 3/23/23.
//

#include "isr.h"
#include "led.h"
#include "mmio.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

void sigint_handler(int sig) {
  printf("Received SIGINT signal\n");
  if (mmio_is_valid()) {
    for (int i = 0; i < 10; i++) {
      set_motor_speed(i, 0);
    }
  }
  set_brightness(100, 000, 100);
  exit(0);
}

int main() {
  int num;
  signal(SIGINT, sigint_handler);
  mmio_init();
  isr_init();
  set_led_status();
  set_brightness(100, 100, 000);
  speed1 = 0;
  while (1) {
    // scanf("%d", &speed1);
  }

  close_mem();
}
