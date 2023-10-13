#include "rover.h"

static steering_motor_t steer_FR;
static steering_motor_t steer_RR;
static steering_motor_t steer_FL;
static steering_motor_t steer_RL;

void rover_init() {
  steer_FR.index = FRS;
  steer_FR.state = STATE_INITIALIZE;

  steer_RR.index = RRS;
  steer_RR.state = STATE_INITIALIZE;

  steer_FL.index = FLS;
  steer_FL.state = STATE_INITIALIZE;

  steer_RL.index = RLS;
  steer_RL.state = STATE_INITIALIZE;

  steering_motor_handle_state(&steer_FR);
  steering_motor_handle_state(&steer_RR);
  steering_motor_handle_state(&steer_FL);
  steering_motor_handle_state(&steer_RL);

  rover_state = ROVER_CALIBRATE_WAITING;
}

int rover_is_calibrated() { return rover_state == ROVER_CALIBRATE_READY; }

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
      rover_steer_forward();
      rover_state = ROVER_CALIBRATE_READY;
    }
    break;
  case ROVER_CALIBRATE_READY:
    break;
  default:
    break;
  }
}

void rover_update_steering() {
  steering_motor_handle_state(&steer_FR);
  steering_motor_handle_state(&steer_RR);
  steering_motor_handle_state(&steer_FL);
  steering_motor_handle_state(&steer_RL);
}

void rover_stop() {
  set_motor_speed(FRW, 0);
  set_motor_speed(RRW, 0);
  set_motor_speed(FLW, 0);
  set_motor_speed(RLW, 0);
  set_motor_speed(MRW, 0);
  set_motor_speed(MLW, 0);
}

void rover_forward(int speed) {
  set_motor_speed(FRW, -speed);
  set_motor_speed(MRW, -speed);
  set_motor_speed(RRW, -speed);
  set_motor_speed(FLW, speed);
  set_motor_speed(MLW, speed);
  set_motor_speed(RLW, speed);
}

void rover_reverse(int speed) {
  set_motor_speed(FRW, speed);
  set_motor_speed(MRW, speed);
  set_motor_speed(RRW, speed);
  set_motor_speed(FLW, -speed);
  set_motor_speed(MLW, -speed);
  set_motor_speed(RLW, -speed);
}

void rover_pointTurn_CW(int speed) {
  set_motor_speed(FRW, speed);
  set_motor_speed(MRW, speed);
  set_motor_speed(RRW, speed);
  set_motor_speed(FLW, speed);
  set_motor_speed(MLW, speed);
  set_motor_speed(RLW, speed);
}

void rover_pointTurn_CCW(int speed) {
  set_motor_speed(FRW, -speed);
  set_motor_speed(MRW, -speed);
  set_motor_speed(RRW, -speed);
  set_motor_speed(FLW, -speed);
  set_motor_speed(MLW, -speed);
  set_motor_speed(RLW, -speed);
}

void rover_steer_forward() {
  steer_FR.target = steer_FR.center_pos + 0;
  steer_FL.target = steer_FL.center_pos + 0;
  steer_RR.target = steer_RR.center_pos + 0;
  steer_RL.target = steer_RL.center_pos + 0;
}

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

void rover_steer_point() {
  steer_FR.target = steer_FR.center_pos - MAX_STEERING_TICKS;
  steer_RR.target = steer_RR.center_pos + MAX_STEERING_TICKS;
  steer_FL.target = steer_FL.center_pos + MAX_STEERING_TICKS;
  steer_RL.target = steer_RL.center_pos - MAX_STEERING_TICKS;
}
