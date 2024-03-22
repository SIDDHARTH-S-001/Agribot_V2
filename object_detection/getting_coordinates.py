from ultralytics import YOLO
import cv2
import math 
import time
import numpy as np

# Load camera matrix and distortion coefficients
camera_matrix = np.loadtxt("camera_matrix.txt")
dist_coeffs = np.loadtxt("distortion_coefficients.txt")

# Load distortion matrix
distortion_matrix = np.loadtxt("distortion_coefficients.txt")

# Start webcam
cap = cv2.VideoCapture(0)

# Model
model = YOLO("yolov8n.pt")

# Object classes to detect
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]

detectNames = ["bottle", "cup", "sports ball"]

# Variables for FPS calculation
prev_time = 0
fps = 0

while True:
    success, img = cap.read()
    results = model(img, stream=True)

    # Coordinates
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # Bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # Convert to int values

            # Confidence
            confidence = math.ceil((box.conf[0]*100))/100
            # Class name
            cls_index = int(box.cls[0])
            cls = classNames[cls_index]

            if cls in detectNames:
                if confidence >= 0.5:  # Check if confidence is above 0.5
                    # Put box in cam
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)   

                    # Undistort the bounding box points
                    pts_src = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
                    pts_dst = cv2.undistortPoints(pts_src.reshape(-1, 1, 2), camera_matrix, dist_coeffs)
                    pts_dst = pts_dst.squeeze()

                    # Perspective transformation
                    pts_3d = cv2.perspectiveTransform(pts_dst.reshape(-1, 1, 2), np.linalg.inv(camera_matrix))
                    pts_3d = pts_3d.squeeze()

                    # Object details
                    org = [int(pts_3d[0][0]), int(pts_3d[0][1])]
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale = 1
                    color = (255, 0, 0)
                    thickness = 2

                    # Display class name
                    cv2.putText(img, cls, org, font, fontScale, color, thickness)

                    # Check if pts_3d has at least 3 elements
                    if len(pts_3d) >= 3:
                        # Display coordinates
                        coord_text = f"x: {int(pts_3d[0][0])}, y: {int(pts_3d[0][1])}, z: {int(pts_3d[0][2])}"
                        cv2.putText(img, coord_text, (org[0], org[1] + 30), font, fontScale, color, thickness)

    # Calculate FPS
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    # Display FPS in the top right corner
    cv2.putText(img, f"FPS: {int(fps)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow('Webcam', img)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

