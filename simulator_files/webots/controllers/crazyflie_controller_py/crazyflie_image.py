import cv2
import numpy as np
from ultralytics import YOLO

# Load the YOLO model
model = YOLO("yolov8n.pt")


def detect_objects_in_frame(frame):
    """
    Perform object detection on the given frame using YOLO and return the annotated frame.
    """
    # Perform object detection with YOLO
    results = model(frame)

    # Annotate the frame with the detection results
    annotated_frame = results[0].plot()

    return annotated_frame
