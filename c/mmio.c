#include "mmio.h"

static volatile unsigned int *mmio;
static uint8_t *mmio_read;

volatile unsigned int *mmio_init() {
  int memfd;
  mmio = 0;
  // Open the /dev/mem device file
  memfd = open("/dev/mem", O_RDWR | O_SYNC);

  // Map the I/O register to the virtual memory space
  mmio = mmap(NULL, getpagesize(), PROT_READ | PROT_WRITE, MAP_SHARED, memfd,
              IOMEM_ADDRESS);

  // Close the /dev/mem device file
  close(memfd);

  if (mmio == MAP_FAILED) {
    perror("Failed to mmap");
    exit(EXIT_FAILURE);
  }

  mmio_read = (uint8_t *)(mmio + READ_OFFSET);

  return (volatile unsigned int *)mmio;
}

int close_mem() {
  // Unmap the virtual memory
  munmap((void *)mmio, getpagesize());
  return 0;
}

uint8_t mmio_is_valid() { return mmio ? 1 : 0; }

void set_PL_register(uint8_t address, uint8_t value) {
  *mmio = (address << ADDRESS_OFFSET) + value;
}

uint8_t get_PL_register(uint8_t address) {
  *mmio = (address << ADDRESS_OFFSET);
  return *mmio_read;
}
