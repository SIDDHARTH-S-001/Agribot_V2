import cv2
from ultralytics import YOLO
from transformers import DPTImageProcessor, DPTForDepthEstimation
import torch
import numpy as np
from PIL import Image

# Load YOLO model
model_yolo = YOLO("yolov5s.pt")

# Load depth estimation model
processor = DPTImageProcessor.from_pretrained("Intel/dpt-swinv2-tiny-256")
model_depth = DPTForDepthEstimation.from_pretrained("Intel/dpt-swinv2-tiny-256")

# Load camera matrix and distortion matrix
camera_matrix = np.loadtxt('camera_matrix.txt')
distortion_matrix = np.loadtxt('distortion_matrix.txt')

# Open webcam
cap = cv2.VideoCapture(0)

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Main loop
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Couldn't read frame.")
        break

    # Object detection with YOLO
    results = model_yolo(frame)

    # Coordinates of the detected object
    object_coords = None

    for r in results.xyxy[0]:
        x1, y1, x2, y2, _ = map(int, r)
        object_coords = ((x1 + x2) / 2, (y1 + y2) / 2)  # Using center of the bounding box

        # Display bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)

        # Display class name
        cv2.putText(frame, model_yolo.names[int(r[-1])], (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

    if object_coords:
        # Convert pixel coordinates to real-world coordinates
        object_coords_homogeneous = np.array([[object_coords[0]], [object_coords[1]], [1]])
        object_coords_undistorted = np.dot(np.linalg.inv(camera_matrix), object_coords_homogeneous)
        object_coords_undistorted /= object_coords_undistorted[2]  # Normalize

        # Depth estimation
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        inputs = processor(images=image, return_tensors="pt")

        with torch.no_grad():
            outputs = model_depth(**inputs)
            predicted_depth = outputs.predicted_depth

        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=image.size[::-1],
            mode="bicubic",
            align_corners=False,
        )
        output = prediction.squeeze().cpu().numpy()
        depth_value = output[int(object_coords[1]), int(object_coords[0])]  # Depth value at object's pixel coordinates

        # Display depth value
        cv2.putText(frame, f'Depth: {depth_value:.2f} meters', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Display the frame
    cv2.imshow('frame', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
