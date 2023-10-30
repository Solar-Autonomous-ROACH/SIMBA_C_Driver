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

  while (!done) {
    printf("settting LED to high\n");
    *mmio = (0x80010000) + 1;

    sleep(1);

    printf("settting LED to low\n");
    *mmio = (0x80010000) + 0;

    sleep(1);
  }

  printf("closing mmio\n");
  close_mem();

  return 0;
}
