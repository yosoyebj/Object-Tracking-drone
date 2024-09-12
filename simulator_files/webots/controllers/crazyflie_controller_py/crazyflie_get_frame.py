import cv2
import numpy as np
from crazyflie_image import detect_objects_in_frame

def get_camera_frame(camera):
    """
    Capture a frame from the Webots camera, convert it to a format suitable for OpenCV,
    perform object detection, and return the annotated image and detected object details.
    """
    # Get the camera image
    image = camera.getImage()

    # Convert the Webots image (raw bytes) to a format OpenCV can use (BGRA to BGR)
    width = camera.getWidth()
    height = camera.getHeight()
    image_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))  # BGRA format
    image_bgr = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)  # Convert to BGR for OpenCV

    # Detect objects in the frame using YOLO
    annotated_frame, object_center_x, object_center_y, object_size, detected_object = detect_objects_in_frame(image_bgr)

    return annotated_frame, object_center_x, object_center_y, object_size, detected_object

def display_camera_frame(image_bgr):
    """
    Display the camera frame using OpenCV.
    """
    cv2.imshow('Drone Camera', image_bgr)
    if cv2.waitKey(1) & 0xFF == ord('p'):  # Press 'p' to quit
        cv2.destroyAllWindows()
        return False  # Indicate that the loop should stop
    return True  # Continue displaying frames
