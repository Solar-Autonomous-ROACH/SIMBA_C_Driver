#include "steering_motor.h"
#include "rover.h"
#include "servo.h"

#define buf_size 20

long pos[buf_size];
long pos_index = 0;
long spinup = 20;
long vel = 0;

void calibrate (steering_motor_t *s_motor) {
    if(s_motor->state == STATE_WAITING)
        s_motor->state = STATE_CALIBRATION_LEFT;
}

int steering_motor_handle_state(steering_motor_t *s_motor) {
    long long current;
    long long target;
    long long difference = 0;


    // TODO: Not defined in rover.h: This function updates the position of a motor with the given index.
    // motor_update(s_motor->index);
    Servo_update(s_motor->servo);
    switch (s_motor->state) {
        case STATE_INITIALIZE:
            //printf("STATE_INITIALIZE\n");
            s_motor->left_pos = 0;
            s_motor->right_pos = 0;
            s_motor->center_pos = 0;
            //set_target_position(s_motor->index, (long long)get_motor_position(s_motor->index) << 32);
            //printf("I: %d, T: %ld, C:%ld\n",s_motor->index, (long)(get_target_position(s_motor->index)>>32), get_motor_position(s_motor->index));
            s_motor->state = STATE_WAITING;
            break;

        case STATE_WAITING:
            //printf("STATE_WAITING\n");
            break;

        case STATE_CALIBRATION_LEFT:
            //printf("STATE_CALIBRATION_LEFT\n");
            //printf("Raw: %ld\n", get_motor_position(s_motor->index));

            // TODO: Not defined in rover.h: Sets the Speed of the motor
            // set_motor_speed(s_motor->index, CALIBRATION_SPEED);
            motor_set_speed(s_motor->servo->motor.addr, CALIBRATION_SPEED);

            //printf("Raw: %d\n", get_raw_pos(s_motor->index));
            // TODO: Not defined in rover.h: Gets the position of the motor
            // pos[pos_index] = get_motor_position(s_motor->index);
            pos[pos_index] = motor_get_position(s_motor->servo->motor.addr);
            //printf("pos: %ld spinup: %ld vel: %ld\n", pos[pos_index], spinup, vel);            

            vel = 0;
            for(int i = 0; i < buf_size-2; i++){
                vel += pos[(buf_size+pos_index-(i+0))%buf_size] - pos[(buf_size+pos_index-(i+1))%buf_size];
            }
            //printf("%ld\n",vel);
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
            //printf("S: %ld\n", vel);
//            current = (long long)get_motor_position(s_motor->index) << 32;
//            target = get_target_position(s_motor->index);
//            target+= (long long)CALIBRATION_SPEED;
//            set_target_position(s_motor->index, target);
//            difference = target - current;
//            int speed = ((KP * difference)>>32) - ( KV * get_motor_velocity(s_motor->index)) ;
//            printf("S: %d, T: %ld, C:%ld\n",speed, (long)(get_target_position(s_motor->index)>>32), get_motor_position(s_motor->index));
//
//
                if(spinup < 1 ) {
                    if (vel < 1) {
                        // TODO: Not defined in rover.h: Gets the position of the motor
                        // s_motor->left_pos = get_motor_position(s_motor->index);;
                        s_motor->left_pos = motor_get_position(s_motor->servo->motor.addr);
                        s_motor->state = STATE_CALIBRATION_RIGHT;
                        spinup = 50;
                        //printf("left: %ld,\n", s_motor->left_pos);
                    }
                }  else {
                    if(vel > 0) spinup--;
                }
            break;


        case STATE_CALIBRATION_RIGHT:
            //printf("STATE_CALIBRATION_RIGHT\n");
            // TODO: Not defined in rover.h: Sets the speed of the motor
            // set_motor_speed(s_motor->index, -CALIBRATION_SPEED);
            motor_set_speed(s_motor->servo->motor.addr, -CALIBRATION_SPEED);
            // TODO: Not defined in rover.h: Gets the position of the motor
            // pos[pos_index] = get_motor_position(s_motor->index);
            pos[pos_index] = motor_get_position(s_motor->servo->motor.addr);

            vel = 0;
            for(int i = 0; i < buf_size-2; i++){
                vel += pos[(buf_size+pos_index-(i+0))%buf_size] - pos[(buf_size+pos_index-(i+1))%buf_size];
            }

            pos_index = (pos_index + 1) % buf_size;

            difference = difference >> 32;

            if(spinup < 1 ) {
                if (vel > -1) {
                    // TODO: Not defined in rover.h: Sets the speed of the motor
                    // set_motor_speed(s_motor->index, 0);
                    motor_set_speed(s_motor->servo->motor.addr, 0);
                    // TODO: Not defined in rover.h: Gets the position of the motor
                    // s_motor->right_pos = get_motor_position(s_motor->index);
                    s_motor->right_pos = motor_get_position(s_motor->servo->motor.addr);
                    s_motor->state = STATE_CALIBRATION_CENTER;

                    //printf("Left: %ld\t Right: %ld\t Center: %ld\t Current: %ld\n", s_motor->left_pos, s_motor->right_pos, s_motor->center_pos, motor_get_position(s_motor->index));

                    s_motor->center_pos = ((s_motor->left_pos + s_motor->right_pos) >> 1);
                }
            }  else {
                if(vel < 0) spinup--;
            }
            break;


        case STATE_CALIBRATION_CENTER:
            //printf("STATE_CALIBRATION_CENTER\n");

            // current = (long long)get_motor_position(s_motor->index) << 32;
            current = (long long)motor_get_position(s_motor->servo->motor.addr) << 32;
            target = ((long long) s_motor->center_pos) << 32;
            // TODO: Not defined in rover.h: Sets the target position of the motor. Remove for now?
            // set_target_position(s_motor->index, target);
            difference = target - current;
            int speed = ((1 * difference)>>32) ;
            speed  = 0;
            if(difference > 1){
                speed = CALIBRATION_SPEED;
                // set_motor_speed(s_motor->index, speed );
                motor_set_speed(s_motor->servo->motor.addr, speed);
            }else if(difference < -1){
                speed = -CALIBRATION_SPEED;
                // set_motor_speed(s_motor->index, speed );
                motor_set_speed(s_motor->servo->motor.addr, speed);
            } else {
                s_motor->target = s_motor->center_pos;
                // set_motor_speed(s_motor->index, 0 );
                motor_set_speed(s_motor->servo->motor.addr, 0);
                s_motor->state = STATE_READY;
            }
            break;

        case STATE_READY:
            //printf("STATE_READY\n");
            // current = get_motor_position(s_motor->index);
            current = motor_get_position(s_motor->servo->motor.addr);
            target = s_motor->target;
            difference = target - current;
            if(difference > 1){
                // set_motor_speed(s_motor->index, 30 );
                motor_set_speed(s_motor->servo->motor.addr, CALIBRATION_SPEED);
            }else if(difference < -1){
                // set_motor_speed(s_motor->index, -30 );
                motor_set_speed(s_motor->servo->motor.addr, -CALIBRATION_SPEED);
            } else {
                // set_motor_speed(s_motor->index, 0 );
                motor_set_speed(s_motor->servo->motor.addr, 0);
            }
            break;

        default:
            return -1;
            break;
    }
    if(s_motor->state == STATE_READY) return 1;
    return 0;
}
