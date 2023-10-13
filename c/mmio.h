//
// Created by Tyler Bovenzi on 3/23/23.
//
#ifndef ROVERCORE_MMIO_H
#define ROVERCORE_MMIO_H

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

// Memory address of the I/O register
#define IOMEM_ADDRESS 0x41200000
#define ADDRESS_OFFSET 8
#define READ_OFFSET 2
#define WATCHDOG_REG 0x50
#define DEBUG_REG 0x80

// Function prototypes
volatile unsigned int *mmio_init(void);

int close_mem();

uint8_t mmio_is_valid();

void set_PL_register(uint8_t address, uint8_t value);

uint8_t get_PL_register(uint8_t address);

#endif // ROVERCORE_MMIO_H
