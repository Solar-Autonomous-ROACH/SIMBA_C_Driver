#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include "isr.h"
#include "move.h"

int MAX_SPEED = 100;

int entered_distance;
int current_distance;
int entered_distance_in_ticks;
int increment = 7;
int last_speed;
int i = 0;

long long error;
static long long ticks_moved_so_far = 0;
long long time_in_ms; // essentially time in ms
int done_moving = 0;

int update_distance(int update) { current_distance += update; }

int finished_moving() { return done_moving; }

// void speed_up(){
//     increment += 100;
// }

// int get_distance_moved(){
//     return ;
// }

// uint16_t ticks_to_distance(){
//     return WHEEL_DIAMETER/TICKS_PER_REV_6;
// }

// uint16_t calc_circumference(){
//     return(2* M_PI * WHEEL_DIAMETER);
// }

long ticks_to_distance(long ticks) {
  long distance =
      (ticks / TICKS_PER_REV_6) * (WHEEL_DIAMETER * M_PI / GEAR_RATIO_6);
  return distance;
}

int distance_to_ticks(int distance) {
  return (distance * TICKS_PER_REV_6) / (WHEEL_DIAMETER * M_PI / GEAR_RATIO_6);
}

int enter_distance() {
  int input_distance;
  printf("Enter a distance: ");
  scanf("%d", &input_distance);
  entered_distance = input_distance;
  entered_distance_in_ticks = distance_to_ticks(input_distance);
  printf("Given distance: %d in mm\t Distance in ticks: %d\n", input_distance,
         entered_distance_in_ticks);
  return 1;
}

int enter_speed_and_distance(int input_distance, long long *inc) {
  int input_speed;
  int speed;
  printf("Enter a distance: ");
  scanf("%d", &input_distance);
  int ticks = distance_to_ticks(input_distance);
  printf("Enter a speed: ");
  scanf("%d", &input_speed);
  speed = (input_distance / input_speed >> 3);
  inc = &speed;
  return ticks;
}

// void rover_move_init(){

// }

int rover_move() {
  long long difference;
  long long speed;
  long long initial_position = get_motor_position(1);
  long long cur_target = get_target_position(1) + ((long long)increment << 32);
  done_moving = 0;

  set_target_position(1, cur_target);
  set_target_position(3, cur_target);
  set_target_position(5, cur_target);
  set_target_position(7, cur_target);
  set_target_position(8, cur_target);
  set_target_position(9, cur_target);

  // ticks_moved_so_far += increment;

  // printf("speed: %llu \ttime: %llu \tsof far: %llu \n", time_in_ms,
  // ticks_moved_so_far);

  // check if movement is complete
  if (ticks_moved_so_far >= entered_distance_in_ticks) {
    increment = 7;
    ticks_moved_so_far = 0;
    time_in_ms = 0;
    speed = 0;
    difference = 0;
    printf("i finished\n");
    done_moving = 1;
    i = 0;
  }

  // calc the difference and speed
  difference =
      get_target_position(1) - ((long long)(get_motor_position(1)) << 30);
  speed = ((KP * difference) >> 32) - (KV * get_motor_velocity(1));
  printf("speed pre change: %llu", speed);
  // speed = 50;
  //  if(speed < 70){
  //      speed++;
  //  }
  motor_update(1);
  motor_update(3);
  motor_update(5);
  motor_update(7);
  motor_update(8);
  motor_update(9);
  // //top off speed at 130
  if (speed > MAX_SPEED) {
    speed = MAX_SPEED;
  }

  // choose a point when to start decelerating
  if (((entered_distance_in_ticks - ticks_moved_so_far) < 200) &&
      (increment > 0)) {
    i++;
    // printf("im deramping: %d distance left %d\n", increment,
    // entered_distance_in_ticks - ticks_moved_so_far);
    increment -= 1;
    // speed = speed - i;

    // speed -= last_speed - 20;
    printf("deramp speed: %llu\t, left: %d\n", speed,
           entered_distance_in_ticks - ticks_moved_so_far);
    // speed -= i;
    if (speed < 0) {
      speed = 20;
    }
  } else if (increment < 20) {
    // printf("incrementing, %d\n", increment);
    increment++;
  }

  // apply the speed to the motors
  set_motor_speed(1, speed);
  set_motor_speed(3, speed);
  set_motor_speed(5, -speed);
  set_motor_speed(7, -speed);
  set_motor_speed(8, -speed);
  set_motor_speed(9, speed);

  // update time and tick counters.
  ticks_moved_so_far += get_motor_position(1) - initial_position;
  time_in_ms++;
  return 0;
}
