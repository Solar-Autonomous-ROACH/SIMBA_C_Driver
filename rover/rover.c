// Rover Control API
#include "rover.h"
#include "mmio.h"
#include "servo.h"

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#define NUM_MOTORS 15
#define ISR_DELAY 1000          // in usec
#define DEFAULT_MOTOR_SPEED 128 // in encoder positions per second
#define ISR_MAX_FUNCS 10 // maximum number of functions to be attached to isr

/** Functions not exposed in header */
void isr(int signum __attribute__((unused)));

Servo servos[NUM_MOTORS];
volatile unsigned int *watchdog_flag;

/** isr globals */
void (*isr_functions[ISR_MAX_FUNCS])(); // Array of ISR function pointers:
unsigned isr_num_functions;

int motor_calibrate(off_t motor_addr) {
  const int TIMEOUT = 1000; // num delays before limit finding timeout
  const int DELAY = 1000;   // usec between checks
  const int SPEED = 64;     // speed to move the motor

  // find the servo that is associated with the motor_addr
  Servo servo;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motor_addr == servos[i].motor.addr) {
      servo = servos[i];
      break;
    }
  }

  int left_limit = 0;
  int right_limit = 0;

  /** Callibrate by finding both limits, and moving motor to midpoint */
  servo.speed = SPEED;
  // RIGHT SIDE: move the motor until the servo count stops incrementing, and
  // mark the limit;
  int last_pos = servo.counts;
  int count = 0;
  while (last_pos != servo.counts) {
    last_pos = servo.counts;
    usleep(DELAY);
    count++;
    if (count > TIMEOUT) {
      return -1;
    }
  }
  right_limit = servo.counts;

  // LEFT SIDE: move the motor until the servo count stops incrementing, and
  // mark the limit;
  servo.speed = -1 * SPEED;
  last_pos = servo.counts;
  count = 0;
  while (last_pos != servo.counts) {
    last_pos = servo.counts;
    usleep(DELAY);
    count++;
    if (count > TIMEOUT) {
      return -1;
    }
  }
  left_limit = servo.counts;

  // set the midpoint
  servo.speed = 0;
  int midpoint = (right_limit - left_limit) / 2;
  servo.setpoint = midpoint;

  // Clear the counter
  servo.motor.clear_enc = 1;
  MotorController_write(&(servo.motor));
  servo.motor.clear_enc = 0;
  MotorController_write(&(servo.motor));

  return -1;
}

/* API functions */
/* Sets the target motor speed.
 * off_t motor_addr: the motor address defined in rover.h
 * int64_t speed: the speed to set the motor in encoder positions per second
 * Return 0 on success, nonzero on failure
 **/
int motor_set_speed(off_t motor_addr, int64_t speed) {
  // given the speed in counts/second, calculate how many counts per isr run
  int counts_per_second = (double)speed * (double)(ISR_DELAY) * 1e-6;
  // find the servo that is associated with the motor_addr
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motor_addr == servos[i].motor.addr) {
      servos[i].speed = counts_per_second;
      return 0;
    }
  }
  return -1;
}

/* Moves the rover forward some distance in +x and backwards in -x distance
 * int64_t dist: the distance to move in encoder positions
 * FIXME: upgrade hardware encoder counters to be 64 bit
 * double speed: the speed to move. should be between 0-255.
 * returns 0 on success, otherwise the number of motors that failed to be
 *updated (side effect) the motor speed stays in effect if the servos are
 *controlled again.
 **/
int rover_move_x(int64_t dist, double speed) {
  int count = 6;
  for (int i = 0; i < NUM_MOTORS; i++) {
    switch (servos[i].motor.addr) {
    case MOTOR_REAR_RIGHT_WHEEL:
    case MOTOR_FRONT_RIGHT_WHEEL:
    case MOTOR_MIDDLE_RIGHT_WHEEL:
      // set max speed
      servos[i].pid.outputLimitMin = -speed;
      servos[i].pid.outputLimitMax = speed;
      // set the distance
      servos[i].setpoint += dist;
      count--;
      break;
    // inverted motors.
    case MOTOR_REAR_LEFT_WHEEL:
    case MOTOR_FRONT_LEFT_WHEEL:
    case MOTOR_MIDDLE_LEFT_WHEEL:
      // set max speed
      servos[i].pid.outputLimitMin = -speed;
      servos[i].pid.outputLimitMax = speed;
      // set the distance
      servos[i].setpoint -= dist;
      count--;
      break;
    default:
      break;
    }
  }
  return count;
}

/*  Return a status that if the rover is currently moving or not.
 * return 1 if done, 0 if any motor is still moving
 * FIXME: will not work because motors are busted!!!
 **/
int check_rover_done() {
  for (int i = 0; i < 10; i++) {
    if (servos[i].setpoint != servos[i].counts) {
      return 0;
    }
  }
  return 1;
}

int isr_init() {
  struct sigaction sa;
  struct itimerval timer;

  // Install the ISR
  sa.sa_handler = (void *)isr;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGALRM, &sa, NULL);

  // Set the timer to trigger every 1ms
  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_usec = ISR_DELAY;
  timer.it_value.tv_sec = 0;
  timer.it_value.tv_usec = ISR_DELAY;
  setitimer(ITIMER_REAL, &timer, NULL);

  // Handle watchdog
  watchdog_flag = mmio_init((off_t)WATCHDOG_REG);

  return 0;
}

// PID Control
void isr(int signum __attribute__((unused))) {
  // Handle watchdog
  *(watchdog_flag) = *(watchdog_flag) ? 0 : 1;

  for (unsigned i = 0; i < isr_num_functions; i++) {
    isr_functions[i]();
  }
}

void rover_isr() {
  // Update servos
  for (int i = 0; i < 10; i++) {
    Servo_update(&servos[i]);
  }

  // Update the setpoitns
  for (int i = 0; i < NUM_MOTORS; i++) {
    servos[i].setpoint += servos[i].speed;
  }
}

int isr_attach_function(void (*fun)()) {
  if (isr_num_functions == ISR_MAX_FUNCS) {
    fprintf(stderr, "Cannot attach any more functions\n");
    return 1; // cannot attach any more functions
  }
  isr_functions[isr_num_functions++] = fun;
  return 0;
}

/**
 * @brief the rover.
 * @return 0 on success, nonzero on failure
 */
int rover_init() {
  // define servos and motors
  off_t motor_addrs[NUM_MOTORS] = {
      MOTOR_REAR_LEFT_WHEEL,   MOTOR_REAR_RIGHT_WHEEL,
      MOTOR_MIDDLE_LEFT_WHEEL, MOTOR_MIDDLE_RIGHT_WHEEL,
      MOTOR_FRONT_LEFT_WHEEL,  MOTOR_FRONT_RIGHT_WHEEL,
      MOTOR_FRONT_LEFT_STEER,  MOTOR_FRONT_RIGHT_STEER,
      MOTOR_REAR_LEFT_STEER,   MOTOR_REAR_RIGHT_STEER,

      // MOTOR_WRIST,
      // MOTOR_BASE,
      // MOTOR_ELBOW,
      // MOTOR_CLAW
      //... more unused motors
  };

  // setup servos
  for (int i = 0; i < 10; i++) {
    // initializing servos
    if (Servo_init(&servos[i], motor_addrs[i], false) != 0) {
      printf("failed to initialize servo%d\n", i);
      return -1;
    }
  }

  if (isr_attach_function(rover_isr) != 0) {
    return 1;
  }
  return 0;
}

int rover_rotate(int dir, int angle) {
  if (angle > 90 || angle < -90) {
    printf("Invalid turn angle\n");
  } else {
    printf("Rover rotating to target angle %d and dir %d\n", angle, dir);
  }
  return 0;
}

/**
 * @brief Closes out all the motor connections
 * @return 0 on success, nonzero on failure
 */
int rover_close() {
  for (int i = 0; i < 10; i++) {
    Servo_close(&servos[i]);
  }
  return 0;
}
