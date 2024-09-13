import random
from controller import Supervisor

# Create a Supervisor instance
supervisor = Supervisor()

# Time step (must match the world time step in Webots)
timestep = int(supervisor.getBasicTimeStep())

# Get a reference to the ball using its DEF name
ball = supervisor.getFromDef("BALL")
translation_field = ball.getField("translation")  # Get the translation field to change the position

# Check if ball is found
if ball is None:
    print("Ball not found!")
else:
    print("Ball found!")

# Variables for movement control
movement_speed = 0.005  # Speed of movement for each update
updates_per_direction = 10  # Number of updates before switching direction
current_update_count = 0  # Track the number of movements in the current direction

# Boundary limits for the environment (adjust according to the environment size)
x_limit = 2.0  # Limit for x-axis movement
y_limit = 2.0  # Limit for y-axis movement (ball can move between y = 0 and y = 2)

# Function to generate new random increments for either x or y axes
def generate_new_direction(axis):
    if axis == 'x':
        return [random.choice([-movement_speed, movement_speed]), 0, 0]  # Move along x-axis
    else:
        return [0, random.choice([-movement_speed, movement_speed]), 0]  # Move along y-axis (up or down)

# Start by moving along the x-axis
movement_direction = generate_new_direction('x')

# Main simulation loop
while supervisor.step(timestep) != -1:
    # Get the ball's current position
    current_position = translation_field.getSFVec3f()

    # Update the ball's position every 0.01 seconds based on the current movement direction
    new_position = [
        current_position[0] + movement_direction[0],  # Increment x position (if moving along x)
        current_position[1] + movement_direction[1],  # Increment y position (if moving along y)
        current_position[2]  # Keep the z position constant
    ]

    # Check if the ball hits the boundaries
    if new_position[0] > x_limit or new_position[0] < -x_limit:
        movement_direction[0] = -movement_direction[0]  # Reverse x direction if out of bounds
        new_position[0] = max(min(new_position[0], x_limit), -x_limit)  # Ensure x stays within limits

    if new_position[1] > y_limit or new_position[1] < 0:  # Ensures the ball doesn't go below the floor
        movement_direction[1] = -movement_direction[1]  # Reverse y direction if out of bounds
        new_position[1] = max(min(new_position[1], y_limit), 0)  # Ensure y stays within limits

    # Apply the new position to the ball
    translation_field.setSFVec3f(new_position)

    # Increment the update counter
    current_update_count += 1

    # After 10 movements, switch the direction (from x to y or from y to x)
    if current_update_count >= updates_per_direction:
        if movement_direction[0] != 0:  # Currently moving along x-axis
            movement_direction = generate_new_direction('y')  # Switch to y-axis
        else:  # Currently moving along y-axis
            movement_direction = generate_new_direction('x')  # Switch to x-axis
        current_update_count = 0  # Reset the update counter

    # Optional: Print the ball's position for debugging
    print(f"Ball position: {new_position}")
