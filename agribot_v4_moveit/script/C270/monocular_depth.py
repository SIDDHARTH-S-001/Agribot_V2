from ultralytics import YOLO
import cv2
import math 
import time
from transformers import DPTImageProcessor, DPTForDepthEstimation
import torch
import numpy as np
from PIL import Image

class MonoCularDepth:
    def __init__(self, camera_matrix_file, distortion_matrix_file):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.obj_model = YOLO("yolov8n.pt")
        self.classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                           "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                           "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                           "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
                           "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
                           "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                           "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                           "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                           "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                           "teddy bear", "hair drier", "toothbrush"]
        # self.detectNames = ["sports ball", "orange", "apple", "backpack"]
        self.detectNames = ["backpack"]
        self.prev_time = 0

        self.camera_matrix = np.loadtxt(camera_matrix_file)
        self.distortion_matrix = np.loadtxt(distortion_matrix_file)

        # Load the model
        self.processor = DPTImageProcessor.from_pretrained("Intel/dpt-swinv2-tiny-256")
        self.depth_model = DPTForDepthEstimation.from_pretrained("Intel/dpt-swinv2-tiny-256")
        self.object_coords = None

    def detect_objects(self):
        while True:
            success, img = self.cap.read()
            results = self.obj_model(img, stream=True)

            for r in results:
                boxes = r.boxes

                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    confidence = math.ceil((box.conf[0]*100))/100
                    cls_index = int(box.cls[0])
                    cls = self.classNames[cls_index]

                    if cls in self.detectNames:
                        if confidence >= 0.5:
                            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)   
                            org = [x1, y1]
                            font = cv2.FONT_HERSHEY_SIMPLEX
                            fontScale = 1
                            color = (255, 0, 0)
                            thickness = 2

                            cv2.putText(img, cls, tuple(org), font, fontScale, color, thickness)

                            # Depth -> center_x and center_y are in pixel values (center of the bounding box) 
                            center_x = (x1 + x2) // 2
                            center_y = (y1 + y2) // 2
                            self.object_coords = (center_x, center_y)

            if self.object_coords is not None:
                # Convert pixel coordinates to real-world coordinates
                object_coords_homogeneous = np.array([[self.object_coords[0]], [self.object_coords[1]], [1]])
                object_coords_undistorted = np.dot(np.linalg.inv(self.camera_matrix), object_coords_homogeneous)
                object_coords_undistorted /= object_coords_undistorted[2]  # Normalize

                # Depth estimation
                image = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                inputs = self.processor(images=image, return_tensors="pt")

                with torch.no_grad():
                    outputs = self.depth_model(**inputs)
                    predicted_depth = outputs.predicted_depth

                prediction = torch.nn.functional.interpolate(
                    predicted_depth.unsqueeze(1),
                    size=image.size[::-1],
                    mode="bicubic",
                    align_corners=False,
                )
                output = prediction.squeeze().cpu().numpy()
                depth_value = output[int(self.object_coords[1]), int(self.object_coords[0])] / 1000 # Depth value at object's pixel coordinates

                # Display depth value
                cv2.putText(img, f'Depth: {depth_value:.2f} meters', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
               
            current_time = time.time()
            fps = 1 / (current_time - self.prev_time)
            self.prev_time = current_time

            cv2.putText(img, f"FPS: {int(fps)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow('Webcam', img)
            if cv2.waitKey(1) == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera_matrix_file = "/home/siddharth/capstone_ws/src/Agribot_V2/agribot_v4_moveit/script/C270/camera_matrix_c270.txt"
    distortion_matrix_file = "/home/siddharth/capstone_ws/src/Agribot_V2/agribot_v4_moveit/script/C270/distortion_coefficients_c270.txt"
    detector = MonoCularDepth(camera_matrix_file, distortion_matrix_file)
    detector.detect_objects()
