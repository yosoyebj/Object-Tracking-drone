def calculate_movement_to_center(object_center_x, object_center_y, frame_center_x, frame_center_y):
    """
    Calculate how much the drone should move to keep the object in the center of the camera frame.
    """
    # Difference between the object's center and the camera's center
    x_diff = object_center_x - frame_center_x
    y_diff = object_center_y - frame_center_y

    # Initialize desired movement
    sideways_desired = 0
    height_diff_desired = 0

    # Threshold to ignore tiny differences
    threshold = 20  # You can tweak this value to make the drone more or less sensitive

    # Horizontal movement (left or right)
    if abs(x_diff) > threshold:
        if x_diff > 0:
            sideways_desired = -0.5  # Move right
        else:
            sideways_desired = 0.5  # Move left

    # Vertical movement (up or down)
    if abs(y_diff) > threshold:
        if y_diff > 0:
            height_diff_desired = -0.1  # Move down
        else:
            height_diff_desired = 0.1  # Move up

    return sideways_desired, height_diff_desired
