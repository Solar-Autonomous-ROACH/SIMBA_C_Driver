#ifndef ISR_H
#define ISR_H

#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>

#include "mmio.h"
#include "rover.h"

// ISR Constants
#define ISR_DELAY 1000      // in usec, DO NOT CHANGE, effects PID
#define ISR_MAX_FUNCS 10    // maximum number of functions to be attached to isr

// ISR Functions
int isr_init();
void isr(int signum __attribute__((unused)));
volatile unsigned int *watchdog_flag;

/** isr globals */
static void (*isr_functions[ISR_MAX_FUNCS])(); // Array of ISR function pointers:
static unsigned isr_num_functions;

#endif // ISR_H