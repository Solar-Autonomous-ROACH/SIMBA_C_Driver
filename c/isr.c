#include "isr.h"
#include "mmio.h"
static int count_ms = 0;
static int state = 10;
static int motor = 0;
static uint64_t total_count = 120000;
static uint8_t watchdog_flag = 0;
int new_inc = 4;

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
  timer.it_interval.tv_usec = 1000; // was 1000
  timer.it_value.tv_sec = 0;
  timer.it_value.tv_usec = 1000; // was 1000
  setitimer(ITIMER_REAL, &timer, NULL);
  set_target_position(0, 0);

  for (int i = 0; i < 14; i++) {
    set_motor_speed(i, 0);
  }

  // rover_init();
  return 0;
}

int isr(int signum) {

  set_PL_register(WATCHDOG_REG, watchdog_flag);
  // set_PL_register(DEBUG_REG, 0xFF);

  switch (state) {
  case 10:
    rover_init();
    // if(enter_distance() == 1){
    //     state++;
    // }
    state++;
    break;
  case 11:
    rover_calibrate();
    if (rover_is_calibrated())
      state++;
    count_ms = 0;
    break;
  case 12:
    rover_steer_point();
    if (count_ms >= 1000) {
      count_ms = 0;
      state++;
    }
    break;
  case 13:
    rover_steer_forward();
    if (count_ms >= 1000) {
      count_ms = 0;
      state++;
    }
    break;
  case 14:
    rover_steer_left(200);
    if (count_ms >= 1000) {
      count_ms = 0;
      state = 12;
    }
    break;

  case 15:
    if (enter_distance() == 1) {
      state++;
    }
    break;
  default:
    // rover_steer_forward();
    //            rover_move();
    //            if( finished_moving() == 1){
    //                state = 15;
    //                for(int i = 0; i<10; i++){
    //                    set_motor_speed(i, 0);
    //                }
    //            }
    // rover_steer_right(200);
    // if(count_ms >= 1000){
    //     count_ms = 0;
    //     state=12;
    // }
    // new_inc++;
    break;
  }

  if (rover_is_calibrated()) {
    rover_update_steering();
  }

  // set_PL_register(DEBUG_REG, 0x00);
  watchdog_flag = !watchdog_flag;
  count_ms++;
  total_count++;
  return 0;
}
