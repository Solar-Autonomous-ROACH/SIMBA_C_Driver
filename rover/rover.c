// Rover Control API
#include "rover.h"
#include "isr.h"
#include "mmio.h"
#include "servo.h"
#include "steering_motor.h"

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#define NUM_MOTORS 15
#define ISR_DELAY 1000          // in usec, DO NOT CHANGE, effects PID
#define DEFAULT_MOTOR_SPEED 128 // in encoder positions per second

// TODO: try making static to put in rover.h
Servo servos[NUM_MOTORS];
volatile unsigned int *watchdog_flag;

// should move this to rover.h
static steering_motor_t steer_FR;

static steering_motor_t steer_RR;
static steering_motor_t steer_FL;
static steering_motor_t steer_RL;

/* API functions */
/* Sets the target motor speed.
 * off_t motor_addr: the motor address defined in rover.h
 * int64_t speed: the speed to set the motor in encoder positions per second
 * Return 0 on success, nonzero on failure
 **/
int motor_set_speed(off_t motor_addr, double speed) {
  // given the speed in counts/second, calculate how many counts per isr run
  //double counts_per_second = (double)speed * (double)(ISR_DELAY)*1e-6;
  // find the servo that is associated with the motor_addr
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motor_addr == servos[i].motor.addr) {
      //servos[i].speed = counts_per_second;
      // set max speed
      servos[i].pid.outputLimitMin = -speed;
      servos[i].pid.outputLimitMax = speed;
      // set the distance
      servos[i].setpoint = 100000;
      return 0;
    }
  }
  return -1;
}

/* Get the current position of the motor
 * Sets the target motor speed.
 * off_t motor_addr: the motor address defined in rover.h
 * Return the current position of the motor in encoder counts
 **/
int64_t motor_get_position(off_t motor_addr) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motor_addr == servos[i].motor.addr) {
      return servos[i].counts;
    }
  }
  return -1;
}
/* Calibrate the rover
 * Return 0 on success, nonzero on failure
*/
int rover_calibrate() {
  // TODO: complete, for now just calibrate one motor
  // calibrate the first motor
  calibrate(&steer_FR);
  return 0;
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
int isr() {
  // Handle watchdog
  *(watchdog_flag) = *(watchdog_flag) ? 0 : 1;

  // handle calibration, only one motor for now
  // TODO: Use FSM for rover calibration, because we cannot callibrate them all simultaneously
  //steering_motor_handle_state(&steer_FR);
  //steering_motor_handle_state(&steer_FL);
  steering_motor_handle_state(&steer_RR);
  //steering_motor_handle_state(&steer_RL);

  // Update servos
  for (int i = 0; i < 10; i++) {
    Servo_update(&servos[i]);
  }

  // Update the setpoitns
  /*    
  for (int i = 0; i < NUM_MOTORS; i++) {
    servos[i].setpoint += servos[i].speed;
  }
  */

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

  // handle calibration
  uint8_t count = 4;
  for (int i = 0; i < NUM_MOTORS; i++) {
    switch (servos[i].motor.addr) {
      case MOTOR_FRONT_LEFT_STEER:
        steer_FL.servo = &servos[i];
        steer_FL.state = STATE_INITIALIZE;
        count--;
        break;
      case MOTOR_FRONT_RIGHT_STEER:
        steer_FR.servo = &servos[i];
        steer_FR.state = STATE_INITIALIZE;
        count--;
        break;
      case MOTOR_REAR_LEFT_STEER:
        steer_RL.servo = &servos[i];
        steer_RL.state = STATE_INITIALIZE;
        count--;
        break;
      case MOTOR_REAR_RIGHT_STEER:
        steer_RR.servo = &servos[i];
        steer_RR.state = STATE_INITIALIZE;
        count--;
        break;
      default:
        break;
    }
  }
  // check if all motors are initialized
  steering_motor_handle_state(&steer_FR);
  steering_motor_handle_state(&steer_RR);
  steering_motor_handle_state(&steer_FL);
  steering_motor_handle_state(&steer_RL);
  
  if(count != 0){
    printf("failed to initialize steering motors\n");
    return -1;
  }

  // initialize calibration
  calibrate(&steer_FR);
  calibrate(&steer_RR);
  calibrate(&steer_FL);
  calibrate(&steer_RL);

  // setup isr
  if (isr_init() != 0) {
    printf("failed to initialize isr\n");
    return -1;
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
