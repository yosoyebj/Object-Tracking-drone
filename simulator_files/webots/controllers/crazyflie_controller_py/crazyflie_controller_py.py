import time
from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard, Camera
from crazyflie_get_frame import get_camera_frame, display_camera_frame
from make_it_center import calculate_movement_to_center
from math import cos, sin
import sys
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1  # Desired height to fly at
DESIRED_OBJECT_SIZE = 1000# This is a hypothetical value for desired object size in the frame

# Define modes and timers
manual_mode = True  # Start in manual mode
auto_mode_enabled = False  # Auto mode starts off
object_detected = False  # Flag to track if an object is detected
last_detection_time = None  # Tracks when the object was last detected
no_detection_duration = 1  # Wait 1 second before switching back to manual mode if no detection

if __name__ == '__main__':

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors with the same setup from the core code (stabilization)
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)

    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)

    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)

    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera")
    camera.enable(timestep)

    keyboard = Keyboard()
    keyboard.enable(timestep)

    past_x_global = 0
    past_y_global = 0
    past_time = robot.getTime()

    PID_CF = pid_velocity_fixed_height_controller()
    height_desired = FLYING_ATTITUDE

    print("\n")
    print("====== Controls =======\n\n")
    print(" The Crazyflie can be controlled from your keyboard!\n")
    print(" All controllable movement is in body coordinates\n")
    print("- Use the up, back, right and left button to move in the horizontal plane\n")
    print("- Use Q and E to rotate around yaw ")
    print("- Use W and S to go up and down\n ")

    # Main loop:
    while robot.step(timestep) != -1:
        # Get and display camera frame with YOLO detection
        annotated_frame, object_center_x, object_center_y, object_size, detected_object = get_camera_frame(camera)
        if not display_camera_frame(annotated_frame):
            break  # Stop the loop if the display window is closed

        current_time = time.time()  # Get the current time

        # Object detection logic with continuous tracking
        if detected_object == "orange" or detected_object == "sports ball" or detected_object == "bird":
            last_detection_time = current_time  # Update the last time an object was detected
            object_detected = True

            # If in manual mode, switch to auto mode
            if manual_mode:
                print("Switching to auto mode for object tracking.")
                manual_mode = False  # Turn off manual control
                auto_mode_enabled = True  # Enable auto mode

            # Auto mode: Calculate how to center the object
            frame_center_x = camera.getWidth() / 2
            frame_center_y = camera.getHeight() / 2
            sideways_desired, height_diff_desired, forward_desired = calculate_movement_to_center(
                object_center_x, object_center_y, frame_center_x, frame_center_y, object_size, DESIRED_OBJECT_SIZE
            )
            yaw_desired = 0  # Minimal yaw movement

        else:
            # If the object is not detected, check the last detection time
            if last_detection_time and (current_time - last_detection_time >= no_detection_duration):
                if object_detected:
                    print("Object lost. Switching back to manual mode after 3 seconds.")
                    object_detected = False
                    auto_mode_enabled = False  # Turn off auto mode
                    manual_mode = True  # Re-enable manual mode

            # In manual mode, reset desired movements when no object is detected
            if manual_mode:
                sideways_desired, height_diff_desired = 0, 0
                forward_desired, yaw_desired = 0, 0

        dt = robot.getTime() - past_time
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        altitude = gps.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global) / dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global) / dt

        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = -v_x_global * sinyaw + v_y_global * cosyaw

        # Only handle manual control if in manual mode
        if manual_mode:
            print("Manual mode: Waiting for user input")
            key = keyboard.getKey()
            while key > 0:
                if key == Keyboard.UP:
                    forward_desired += 0.5
                elif key == Keyboard.DOWN:
                    forward_desired -= 0.5
                elif key == Keyboard.RIGHT:
                    sideways_desired -= 0.5
                elif key == Keyboard.LEFT:
                    sideways_desired += 0.5
                elif key == ord('Q'):
                    yaw_desired = +1
                elif key == ord('E'):
                    yaw_desired = -1
                elif key == ord('W'):
                    height_diff_desired += 0.1
                elif key == ord('S'):
                    height_diff_desired -= 0.1
                key = keyboard.getKey()

        # Update desired height based on movement calculation
        height_desired += height_diff_desired * dt

        # PID control to adjust the drone’s movement
        motor_power = PID_CF.pid(dt, forward_desired, sideways_desired,
                                 yaw_desired, height_desired,
                                 roll, pitch, yaw_rate,
                                 altitude, v_x, v_y)

        # Apply calculated velocities to motors
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = gps.getValues()[0]
        past_y_global = gps.getValues()[1]
