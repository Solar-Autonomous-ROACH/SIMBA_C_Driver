// Rover Control API
#include "rover.h"
#include "mmio.h"
#include "servo.h"
#include "steering_motor.h"

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#define NUM_MOTORS 15
#define ISR_DELAY 1000          // in usec, DO NOT CHANGE, effects PID
#define DEFAULT_MOTOR_SPEED 128 // in encoder positions per second
#define ISR_MAX_FUNCS 10 // maximum number of functions to be attached to isr

/** Functions not exposed in header */
void isr(int signum __attribute__((unused)));

// TODO: try making static to put in rover.h
Servo servos[NUM_MOTORS];
volatile unsigned int *watchdog_flag;
static rover_state_t rover_state;

// should move this to rover.h
static steering_motor_t steer_FR;
static steering_motor_t steer_RR;
static steering_motor_t steer_FL;
static steering_motor_t steer_RL;
/** isr globals */
void (*isr_functions[ISR_MAX_FUNCS])(); // Array of ISR function pointers:
unsigned isr_num_functions;

/* API functions */
/* Sets the target motor speed.
 * off_t motor_addr: the motor address defined in rover.h
 * int64_t speed: the speed to set the motor in encoder positions per second
 * Return 0 on success, nonzero on failure
 **/
int motor_set_speed(off_t motor_addr, double speed) {
  // given the speed in counts/second, calculate how many counts per isr run
  // double counts_per_second = (double)speed * (double)(ISR_DELAY)*1e-6;
  // find the servo that is associated with the motor_addr
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motor_addr == servos[i].motor.addr) {
      // servos[i].speed = counts_per_second;
      //  set max speed
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

/* Convert encoder ticks to distance
 * long ticks: the number of encoder ticks
 * Return the distance in mm
 **/
long ticks_to_distance(long ticks){
    long distance = (ticks / TICKS_PER_REV_6) * (WHEEL_DIAMETER * M_PI / GEAR_RATIO_6);
    return distance;
}

/* Convert distance to encoder ticks
 * long distance: the distance in mm
 * Return the number of encoder ticks
 **/
int distance_to_ticks(int distance){
    return (distance * TICKS_PER_REV_6) / (WHEEL_DIAMETER * M_PI / GEAR_RATIO_6);
}

/* Moves the rover forward some distance in +x and backwards in -x distance
 * int dist: the distance to move in mm
 * double speed: the speed to move. should be between 0-255.
 * returns 0 on success, otherwise the number of motors that failed to be
 *updated (side effect) the motor speed stays in effect if the servos are
 *controlled again.
 **/
int rover_move_x(int dist, double speed) {
  long dist_tics = distance_to_ticks(dist);
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
      servos[i].setpoint += dist_tics;
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
      servos[i].setpoint -= dist_tics;
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
  int64_t current_FR = motor_get_position(steer_FR.servo->motor.addr);
  int64_t current_FL = motor_get_position(steer_FR.servo->motor.addr);
  int64_t current_RR = motor_get_position(steer_FR.servo->motor.addr);
  int64_t current_RL = motor_get_position(steer_FR.servo->motor.addr);

  if (steer_FR.target == current_FR) return 1;
  if (steer_FL.target == current_FL) return 1;
  if (steer_RR.target == current_RR) return 1;
  if (steer_RL.target == current_RL) return 1;
  return 0;
}

/* UNTESTED */
void rover_update_steering() {
  steering_motor_handle_state(&steer_FR);
  steering_motor_handle_state(&steer_RR);
  steering_motor_handle_state(&steer_FL);
  steering_motor_handle_state(&steer_RL);
}

/* UNTESTED */
void rover_stop() {
  motor_set_speed(FRW, 0);
  motor_set_speed(RRW, 0);
  motor_set_speed(FLW, 0);
  motor_set_speed(RLW, 0);
  motor_set_speed(MRW, 0);
  motor_set_speed(MLW, 0);
}

/* UNTESTED */
void rover_forward(int speed) {
  motor_set_speed(FRW, -speed);
  motor_set_speed(MRW, -speed);
  motor_set_speed(RRW, -speed);
  motor_set_speed(FLW, speed);
  motor_set_speed(MLW, speed);
  motor_set_speed(RLW, speed);
}

/* UNTESTED */
void rover_reverse(int speed) {
  motor_set_speed(FRW, speed);
  motor_set_speed(MRW, speed);
  motor_set_speed(RRW, speed);
  motor_set_speed(FLW, -speed);
  motor_set_speed(MLW, -speed);
  motor_set_speed(RLW, -speed);
}

/* UNTESTED */
void rover_pointTurn_CW(int speed) {
  motor_set_speed(FRW, speed);
  motor_set_speed(MRW, speed);
  motor_set_speed(RRW, speed);
  motor_set_speed(FLW, speed);
  motor_set_speed(MLW, speed);
  motor_set_speed(RLW, speed);
}

/* UNTESTED */
void rover_pointTurn_CCW(int speed) {
  motor_set_speed(FRW, -speed);
  motor_set_speed(MRW, -speed);
  motor_set_speed(RRW, -speed);
  motor_set_speed(FLW, -speed);
  motor_set_speed(MLW, -speed);
  motor_set_speed(RLW, -speed);
}

/* UNTESTED */
void rover_steer_forward() {
  steer_FR.target = steer_FR.center_pos + 0;
  steer_FL.target = steer_FL.center_pos + 0;
  steer_RR.target = steer_RR.center_pos + 0;
  steer_RL.target = steer_RL.center_pos + 0;
}

/* UNTESTED */
void rover_steer_right(int angle) {
  if (angle > MAX_STEERING_TICKS)
    angle = MAX_STEERING_TICKS;
  if (angle < -MAX_STEERING_TICKS)
    angle = -MAX_STEERING_TICKS;
  steer_FR.target = steer_FR.center_pos + angle;
  steer_RR.target = steer_RR.center_pos - angle;
  steer_FL.target = steer_FL.center_pos + angle;
  steer_RL.target = steer_RL.center_pos - angle;
}

/* UNTESTED */
void rover_steer_left(int angle) {
  if (angle > MAX_STEERING_TICKS)
    angle = MAX_STEERING_TICKS;
  if (angle < -MAX_STEERING_TICKS)
    angle = -MAX_STEERING_TICKS;
  steer_FR.target = steer_FR.center_pos - angle;
  steer_RR.target = steer_RR.center_pos + angle;
  steer_FL.target = steer_FL.center_pos - angle;
  steer_RL.target = steer_RL.center_pos + angle;
}

/* UNTESTED */
void rover_steer_point() {
  steer_FR.target = steer_FR.center_pos - MAX_STEERING_TICKS;
  steer_RR.target = steer_RR.center_pos + MAX_STEERING_TICKS;
  steer_FL.target = steer_FL.center_pos + MAX_STEERING_TICKS;
  steer_RL.target = steer_RL.center_pos - MAX_STEERING_TICKS;
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

  // handle calibration, only one motor for now
  for (unsigned i = 0; i < isr_num_functions; i++) {
    isr_functions[i]();
  }
}

void rover_isr() {
  // Update servos
  if (rover_is_calibrated() == false)
    rover_calibrate();

  // Update the steering
  rover_update_steering();

  // Update servos
  for (int i = 0; i < 10; i++)
    Servo_update(&servos[i]);
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
  if (count != 0) {
    printf("failed to initialize steering motors\n");
    return -1;
  }

  // check if all motors are initialized
  steering_motor_handle_state(&steer_FR);
  steering_motor_handle_state(&steer_RR);
  steering_motor_handle_state(&steer_FL);
  steering_motor_handle_state(&steer_RL);

  // initialize calibration
  rover_state = ROVER_CALIBRATE_WAITING;

  if (isr_attach_function(rover_isr) != 0) {
    fprintf(stderr, "Failed to attach rover_isr\n");
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

int rover_is_calibrated() { return rover_state == ROVER_CALIBRATE_READY; }

/**
 * @brief Calibrates every motor iteratively
 * @return 0 on success, nonzero on failure
 */
void rover_calibrate() {
  switch (rover_state) {
  case ROVER_CALIBRATE_WAITING:
    calibrate(&steer_FR);
    rover_state = ROVER_CALIBRATE_FR;
    break;
  case ROVER_CALIBRATE_FR:
    if (steering_motor_handle_state(&steer_FR)) {
      calibrate(&steer_RR);
      rover_state = ROVER_CALIBRATE_RR;
    }
    break;
  case ROVER_CALIBRATE_RR:
    if (steering_motor_handle_state(&steer_RR)) {
      calibrate(&steer_FL);
      rover_state = ROVER_CALIBRATE_FL;
    }
    break;
  case ROVER_CALIBRATE_FL:
    if (steering_motor_handle_state(&steer_FL)) {
      calibrate(&steer_RL);
      rover_state = ROVER_CALIBRATE_RL;
    }
    break;
  case ROVER_CALIBRATE_RL:
    if (steering_motor_handle_state(&steer_RL)) {
      // rover_steer_forward();
      rover_state = ROVER_CALIBRATE_READY;
    }
    break;
  case ROVER_CALIBRATE_READY:
    break;
  default:
    break;
  }
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
