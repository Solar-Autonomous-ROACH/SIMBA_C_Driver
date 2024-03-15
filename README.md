# SIMBA C Driver

### Quickstart

``` bash
# clone the directory onto Kria board (assumes kria is properly set up with ubuntu)
git clone https://github.com/Solar-Autonomous-ROACH/SIMBA_C_Driver.git

# load the bitstream
sudo su - # must be root to load bitstream
cd SIMBA_C_Driver/bitstream
python load.py
exit # leave root

# compile rover library
cd SIMBA_C_Driver/rover
make

# compile test code and run
cd ../test
make
sudo ./main
```



### Structure

- `/bitstream` contains up-to-date bitstreams for motor controllers on FPGA. See Vivado project in digital design repository in [Motor_Driver](https://github.com/Solar-Autonomous-ROACH/Motor_Driver)

- `/rover` contains the API and code for controlling individual motors, as well as high level functions for moving the rover.
- `/test` provides an example for loading the `librover.a` library and moving the rover with it.



### Bitstreams

More details on underlying digital design located in the digital design repository in [Motor_Driver](https://github.com/Solar-Autonomous-ROACH/Motor_Driver)

**Loading Motor Driver Bitstream:**

To load the motor driver bitstream, you must first be logged in as root:

`$ sudo su -`

Then load the bitstream with `load.py`

``` bashcd bitstream
$ sudo su -
$ cd bitstream
$ python load.py
$ exit // gets out of root
```



### Rover API

The rover API can be exported as `librover.a` and imported by higher level code to move the rover and control individual motors.

**Files**

- `librover.a` : generated library file when `make` is run. Should be imported to system code to control rover.
- `main.c` : legacy code to demo rover before library. Can be removed
- `mmio.c`:  For interacting with FPGA via memory mapped IO.
  - `volatile unsigned int *mmio_init(off_t iomem_address) `:
    - `iomem_address`: IO address associated with a specific AXI module in the digital design.
    - returns a pointer to the virtual memory address that points to the IO address associated with a specific AXI module in the digital design.
  - `close_mem`
  - `mmio_is_valid`
- `motor.c`: For low level motor control functions, used by `pid.c` and `servo.c`
  - `int MotorController_init(MotorController *motor, off_t mmio_address) `: populate MotorController struct, given:
    - `motor`: the MotorController struct to be initialized
    -  `mmio_address` the address of the motor controller AXI module in teh digital design.
    - returns 0 if successful
  - `void MotorController_close(MotorController *motor)`: disable the motor controller
  - `void MotorController_write(MotorController *motor)`: write to mmio
  - `void MotorController_read(MotorController *motor)`: read the mmio
- `pid.c`: Runs PID loop. Best not to mess with this unless you know what you are doing.
- `rover.c`: Rover control API
  - **Low Level Motor Control Functions**
    - `int motor_set_speed(off_t motor_addr, int speed)`: Set the target motor speed
      - `off_t motor_addr`: the motor address defined in rover.h
      - `int speed`: The speed to set the motor in encoder ticks per second.
      - Returns 0 on success, anything else on failure
    - `int64_t motor_get_position(off_t motor_addr):` 
      - `off_t motor_addr`: the motor address degined in rover.h
      - returns: The current position of the motor in encoder ticks
  - **Utilities**
    - `long ticks_to_distance(long ticks)`: convert encoder ticks to distance in mm
      - `long ticks`: the number of encoder ticks
      - returns number of millimeters
    - `int distance_to_ticks(int distance)`: Convert distance in millimeters to encoder ticks
      - `int distance`: distance in mm
      - returns distance in encoder ticks
  - **Rover API Functions**
    - `int rover_init()`
    - `int rover_close()`
    - `void rover_isr()`: The ISR function to update the rover. Added to the ISR with `isr_attach_function`
    - `void rover_calibrate()`: Start rover callibration
    - `int rover_is_calibrated()`: Return true if callibrate, zero otherwise
    - `int rover_move_x(int dist, double speed)`: Move the rover a given distance at a given speed
    - `int check_rover_done()`: Check whether the rover is finished with its last command
    - `rover_update_steering()`: Update the rovers steering motors:
    - `void rover_stop()`: Stop the rover and set to distance control not speed control.
    - `void rover_forward(int speed)`: Move the rover forward unlimited distance at a given speed.
    - `void rover_reverse(int speed) `: Move the rover reverse unlimited distance at a given speed
    - `void rover_pointTurn_CW(int speed)`: Turn clockwise
    - `void rover_pointTurn_CCW(int speed)`: Turn counterclockwise
    - `void rover_steer_forward()`: Steer the wheels to move forward.
    - `void rover_steer_right(int angle)`: Steer the given encoder ticks right.
    - `void rover_steer_left(int angle)`: Steer the given encoder ticks left
    - `void rover_steer_point()`: Move all the steering motors for a point turn.
    - `int rover_rotate(int dir, int angle)`: Not implemented, but rotate the rover a given number of degrees.
    - `int isr_attach_function(void (*fun)())`: Attach a new function to be run by the ISR every 1000 ms.
  - **Should be in isr.c, but are in rover.c ...**
    - `int isr_init()`: Initialize the ISR
    - `void isr(int signum __attribute__((unused)))` : The isr function run every 1000ms
- `servo.c` :
  - ``int Servo_init(Servo *servo, off_t mmio_address, bool inverted)`: Initialize the servo struct
  - `void Servo_close(Servo *servo)`: Close the servo
  - `void Servo_update(Servo *servo)`: Update the servo
- `steering_motor.c`:
  - `void calibrate (steering_motor_t *s_motor)`:
    - Set the steering motor to start calibration
  - `int steering_motor_handle_state(steering_motor_t *s_motor)`:
    - Handle the steering motor state



