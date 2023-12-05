#ifndef ROVERCORE_MMIO_H
#define ROVERCORE_MMIO_H

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

// Function prototypes
volatile unsigned int *mmio_init(off_t iomem_address);

void close_mem(volatile unsigned int *mmio);

uint8_t mmio_is_valid(volatile unsigned int *mmio);

#endif
