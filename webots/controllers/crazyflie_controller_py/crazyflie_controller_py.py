"""crazyflie_controller_py controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Keyboard

import sys
sys.path.append('../../../controllers/')
from  pid_controller import init_pid_attitude_fixed_height_controller, pid_attitude_fixed_height_controller
from pid_controller import MotorPower_t
robot = Robot()

timestep = 16 ##int(robot.getBasicTimeStep())

## Initialize motors
m1_motor = robot.getDevice("m1_motor");
m1_motor.setPosition(float('inf'))
m1_motor.setVelocity(-1)
m2_motor = robot.getDevice("m2_motor");
m2_motor.setPosition(float('inf'))
m2_motor.setVelocity(-1)
m3_motor = robot.getDevice("m3_motor");
m3_motor.setPosition(float('inf'))
m3_motor.setVelocity(-1)
m4_motor = robot.getDevice("m4_motor");
m4_motor.setPosition(float('inf'))
m4_motor.setVelocity(-1)

## Initialize Sensors
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
Keyboard().enable(timestep)


## Initialize variables
rollActual, pitchActual, yawActual, altitudeActual = 0.0, 0.0, 0.0, 0.0
rollDesired, pitchDesired, yawDesired, altitudeDesired = 0.0, 0.0, 0.0, 0.0
past_time = robot.getTime()

## Initialize PID gains.
kp_att_y = 1
kd_att_y = 0.5
kp_att_rp =0.5
kd_att_rp = 0.1
kp_z = 10
ki_z = 50
kd_y = 5
init_pid_attitude_fixed_height_controller();

## Initialize struct for motor power
motorPower = MotorPower_t()

##pid_controller.pid_attitude_fixed_height_controller(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,motor_power)
#print(motor_power.m1)
# Main loop:
while robot.step(timestep) != -1:

    dt = robot.getTime() - past_time;

    ## Get measurements
    rollActual = imu.getRollPitchYaw()[0];
    pitchActual = imu.getRollPitchYaw()[1];
    yawActual = imu.getRollPitchYaw()[2];
    altitudeActual = gps.getValues()[2];

    ## Initialize values
    rollDesired = 0;
    pitchDesired = 0;
    yawDesired = 0;
    altitudeDesired = 1;

    key = Keyboard().getKey()
    while key>0:
        if key == Keyboard().WB_KEYBOARD_UP:
            pitchDesired += 0.05
        elif key == Keyboard().WB_KEYBOARD_DOWN:
            pitchDesired -= 0.05
        elif key == Keyboard().WB_KEYBOARD_RIGHT:
            rollDesired += 0.05
        elif key == Keyboard().WB_KEYBOARD_LEFT:
            rollDesired -= 0.05
        else:
            pitchDesired = 0;
            pitchDesired = 0;


        key = Keyboard().getKey()

    ## PID attitude controller with fixed height
    pid_attitude_fixed_height_controller(rollActual, pitchActual, yawActual, altitudeActual, 
    rollDesired, pitchDesired, yawDesired, altitudeDesired,
     kp_att_rp,  kd_att_rp,  kp_att_y,  kd_att_y,  kp_z,  kd_y,  ki_z, dt, motorPower);

    print(motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4)

    m1_motor.setVelocity(motorPower.m1)
    m2_motor.setVelocity(motorPower.m2)
    m3_motor.setVelocity(motorPower.m3)
    m4_motor.setVelocity(motorPower.m4)
    
    past_time = robot.getTime()

    pass

