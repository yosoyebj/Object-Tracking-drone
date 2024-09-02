from statsmodels.graphics.tukeyplot import results
from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")

video_path = "/home/ebj/Documents/crazyflie-simulation/docs/images/ball.mp4"
video = cv2.VideoCapture("/home/ebj/Documents/crazyflie-simulation/docs/images/ball.mp4")

width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

fps = video.get(cv2.CAP_PROP_FPS)

output_video_path = "annoted_output_video.mp4"
fourcc= cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

while True:
    ret, frame = video.read()
    if not ret:
        break



    results = model(frame)
    annotated_frame = results[0].plot()
    out.write(annotated_frame)
    cv2.imshow("annotated Video", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
out.release()
cv2.destroyAllWindows()