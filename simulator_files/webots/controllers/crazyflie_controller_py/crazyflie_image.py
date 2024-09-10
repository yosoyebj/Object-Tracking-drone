import cv2
import numpy as np
from ultralytics import YOLO

# Load the YOLO model
model = YOLO("yolov8n.pt")

def detect_objects_in_frame(frame):
    """
    Perform object detection on the given frame using YOLO and return the annotated frame,
    the object center, and the type of detected object.
    """
    # Perform object detection with YOLO
    results = model(frame)

    # Annotate the frame with the detection results
    annotated_frame = results[0].plot()

    # Initialize default values for object detection
    object_center_x = frame.shape[1] / 2  # Default to center
    object_center_y = frame.shape[0] / 2  # Default to center
    detected_object = None  # No object detected by default

    # Check if an object is detected and if it matches the desired categories
    if len(results[0].boxes) > 0:
        for box in results[0].boxes:
            label = results[0].names[int(box.cls)]  # Get the object label (class)

            # Check if the detected object is an "orange ball" or "sports ball"
            if "orange" in label or "sports ball" in label:
                x_min = box.xyxy[0][0].item()  # Extracting float value from tensor
                y_min = box.xyxy[0][1].item()
                x_max = box.xyxy[0][2].item()
                y_max = box.xyxy[0][3].item()

                # Calculate the center of the bounding box
                object_center_x = (x_min + x_max) / 2
                object_center_y = (y_min + y_max) / 2
                detected_object = label  # Set the detected object to the label (orange ball or sports ball)
                break  # Break after finding the first relevant object

    return annotated_frame, object_center_x, object_center_y, detected_object
