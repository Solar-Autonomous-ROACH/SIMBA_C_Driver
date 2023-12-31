#include "steering_motor.h"

#define buf_size 20

long pos[buf_size];
long pos_index = 0;
long spinup = 20;
long vel = 0;
void calibrate(steering_motor_t *s_motor) {
  if (s_motor->state == STATE_WAITING)
    s_motor->state = STATE_CALIBRATION_LEFT;
}

int steering_motor_handle_state(steering_motor_t *s_motor) {
  long long current;
  long long target;
  long long difference;

  motor_update(s_motor->index);
  switch (s_motor->state) {
  case STATE_INITIALIZE:
    s_motor->left_pos = 0;
    s_motor->right_pos = 0;
    s_motor->center_pos = 0;
    // set_target_position(s_motor->index, (long
    // long)get_motor_position(s_motor->index) << 32); printf("I: %d, T: %ld,
    // C:%ld\n",s_motor->index, (long)(get_target_position(s_motor->index)>>32),
    // get_motor_position(s_motor->index));
    s_motor->state = STATE_WAITING;
    break;

  case STATE_WAITING:
    break;

  case STATE_CALIBRATION_LEFT:
    // printf("Raw: %ld\n", get_motor_position(s_motor->index));
    set_motor_speed(s_motor->index, CALIBRATION_SPEED);
    // printf("Raw: %d\n", get_raw_pos(s_motor->index));
    pos[pos_index] = get_motor_position(s_motor->index);

    vel = 0;
    for (int i = 0; i < buf_size - 2; i++) {
      vel += pos[(buf_size + pos_index - (i + 0)) % buf_size] -
             pos[(buf_size + pos_index - (i + 1)) % buf_size];
    }
    // printf("%ld\n",vel);
    //            for(int i = 0; i < 10; i++){
    //                if(i == pos_index) printf("_");
    //                printf("%ld\t", pos[i]);
    //            }
    pos_index = (pos_index + 1) % buf_size;
    //

    //            current = (long long)get_motor_position(s_motor->index) << 32;
    //            target = get_target_position(s_motor->index);
    //            target+= (long long)CALIBRATION_SPEED;
    //            set_target_position(s_motor->index, target);
    //            difference = target - current;
    difference = difference >> 32;
    // printf("S: %ld\n", vel);
    //            current = (long long)get_motor_position(s_motor->index) << 32;
    //            target = get_target_position(s_motor->index);
    //            target+= (long long)CALIBRATION_SPEED;
    //            set_target_position(s_motor->index, target);
    //            difference = target - current;
    //            int speed = ((KP * difference)>>32) - ( KV *
    //            get_motor_velocity(s_motor->index)) ; printf("S: %d, T: %ld,
    //            C:%ld\n",speed,
    //            (long)(get_target_position(s_motor->index)>>32),
    //            get_motor_position(s_motor->index));
    //
    //
    if (spinup < 1) {
      if (vel < 1) {
        s_motor->left_pos = get_motor_position(s_motor->index);
        ;
        s_motor->state = STATE_CALIBRATION_RIGHT;
        spinup = 50;
        // printf("left: %ld,\n", s_motor->left_pos);
      }
    } else {
      if (vel > 0)
        spinup--;
    }
    break;

  case STATE_CALIBRATION_RIGHT:
    set_motor_speed(s_motor->index, -CALIBRATION_SPEED);
    pos[pos_index] = get_motor_position(s_motor->index);

    vel = 0;
    for (int i = 0; i < buf_size - 2; i++) {
      vel += pos[(buf_size + pos_index - (i + 0)) % buf_size] -
             pos[(buf_size + pos_index - (i + 1)) % buf_size];
    }

    pos_index = (pos_index + 1) % buf_size;

    difference = difference >> 32;

    if (spinup < 1) {
      if (vel > -1) {
        set_motor_speed(s_motor->index, 0);
        s_motor->right_pos = get_motor_position(s_motor->index);
        ;
        s_motor->state = STATE_CALIBRATION_CENTER;

        // printf("Left: %ld\t Right: %ld\t Center: %ld\t Current: %ld\n",
        // s_motor->left_pos, s_motor->right_pos, s_motor->center_pos,
        // get_motor_position(s_motor->index));

        s_motor->center_pos = ((s_motor->left_pos + s_motor->right_pos) >> 1);
      }
    } else {
      if (vel < 0)
        spinup--;
    }
    break;

  case STATE_CALIBRATION_CENTER:

    current = (long long)get_motor_position(s_motor->index) << 32;
    target = ((long long)s_motor->center_pos) << 32;
    set_target_position(s_motor->index, target);
    difference = target - current;
    int speed = ((1 * difference) >> 32);
    speed = 0;
    if (difference > 1) {
      speed = 30;
      set_motor_speed(s_motor->index, speed);
    } else if (difference < -1) {
      speed = -30;
      set_motor_speed(s_motor->index, speed);
    } else {
      s_motor->target = s_motor->center_pos;
      set_motor_speed(s_motor->index, 0);
      s_motor->state = STATE_READY;
    }
    break;

  case STATE_READY:
    current = get_motor_position(s_motor->index);
    target = s_motor->target;
    difference = target - current;
    if (difference > 1) {
      set_motor_speed(s_motor->index, 30);
    } else if (difference < -1) {
      set_motor_speed(s_motor->index, -30);
    } else {
      set_motor_speed(s_motor->index, 0);
    }
    break;

  default:
    return -1;
    break;
  }
  if (s_motor->state == STATE_READY)
    return 1;
  return 0;
}
