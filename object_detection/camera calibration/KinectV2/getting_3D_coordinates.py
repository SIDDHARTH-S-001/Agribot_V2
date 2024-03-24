import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math 
import time
from ultralytics import YOLO

class KinectDepthProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/kinect2/hd/image_depth_rect', Image, self.depth_callback)
        self.rgb_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.rgb_callback)
        self.camera_info_sub = rospy.Subscriber('/your/rgb/camera_info/topic', CameraInfo, self.camera_info_callback)
        self.camera_info = None
        self.depth_image = None
        self.rgb_image = None
        self.camera_matrix = None
        self.distortion_matrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.model = YOLO("yolov8n.pt")  # Initialize YOLO model
        self.classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
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
        self.detectNames = ["bottle", "cup", "sports ball"]
        self.prev_time = 0
        self.fps = 0

    def load_camera_parameters(self, camera_matrix_file, distortion_matrix_file):
        # Load camera matrix and distortion coefficients from text files
        self.camera_matrix = np.loadtxt(camera_matrix_file)
        self.distortion_matrix = np.loadtxt(distortion_matrix_file)
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg)

    def pixel_to_point(self, u, v, depth):
        if self.camera_matrix is None:
            return None

        if depth == 0:
            return None

        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth

        return x, y, z

    def process_depth_image(self, img):
        if self.depth_image is None or self.camera_matrix is None:
            return

        height, width = self.depth_image.shape[:2]

        depth_image_float = self.depth_image.astype(np.float32)

        for r in self.results:
            boxes = r.boxes

            for box in boxes:
                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # Convert to int values

                # Class name
                cls_index = int(box.cls[0])
                cls = self.classNames[cls_index]

                if cls in self.detectNames:
                    # Confidence
                    confidence = math.ceil((box.conf[0] * 100)) / 100

                    if confidence >= 0.5:  # Check if confidence is above 0.5
                        # Put box in cam
                        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                        # Object details
                        org = [x1, y1]
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        fontScale = 1
                        color = (255, 0, 0)
                        thickness = 2

                        cv2.putText(img, cls, org, font, fontScale, color, thickness)

                        # Depth
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        depth = depth_image_float[center_y, center_x]
                        if depth != 0:
                            # Calculate world coordinates
                            x, y, z = self.pixel_to_point(center_x, center_y, depth)
                            print(f"Object: {cls}, Depth: {depth}, Coordinates: ({x}, {y}, {z})")

    def detect_objects(self, img):
        self.results = self.model(img, stream=True)
        self.process_depth_image(img)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.rgb_image is not None and self.depth_image is not None:
                img = self.rgb_image.copy()
                self.detect_objects(img)

                # Calculate FPS
                current_time = time.time()
                self.fps = 1 / (current_time - self.prev_time)
                self.prev_time = current_time

                # Display FPS in the top right corner
                cv2.putText(img, f"FPS: {int(self.fps)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                            cv2.LINE_AA)

                cv2.imshow('RGB Image', img)
                if cv2.waitKey(1) == ord('q'):
                    break

            rate.sleep()


def main():
    rospy.init_node('kinect_depth_processor')
    depth_processor = KinectDepthProcessor()

    # Specify the paths to camera matrix and distortion matrix text files
    camera_matrix_file = 'camera_matrix_kinect.txt'
    distortion_matrix_file = 'distortion_coefficients_kinect.txt'
    depth_processor.load_camera_parameters(camera_matrix_file, distortion_matrix_file)

    depth_processor.run()

if __name__ == '__main__':
    main()

