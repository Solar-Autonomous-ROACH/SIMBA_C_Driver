#include "mmio.h"

volatile unsigned int *mmio_init(off_t iomem_address) {
  // Open the /dev/mem device file
  int memfd = open("/dev/mem", O_RDWR | O_SYNC);

  // Map the I/O register to the virtual memory space
  volatile unsigned int *mmio =
      mmap(NULL, getpagesize(), PROT_READ | PROT_WRITE, MAP_SHARED, memfd,
           iomem_address);

  // Close the /dev/mem device file
  close(memfd);

  if (mmio == MAP_FAILED) {
    perror("Failed to mmap");
    exit(EXIT_FAILURE);
  }

  return (volatile unsigned int *)mmio;
}

void close_mem(volatile unsigned int *mmio) {
  // Unmap the virtual memory
  munmap((void *)mmio, getpagesize());
}

uint8_t mmio_is_valid(volatile unsigned int *mmio) { return mmio ? 1 : 0; }
