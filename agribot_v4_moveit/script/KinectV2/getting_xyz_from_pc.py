import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from ultralytics import YOLO

class KinectDepthProcessor:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = rospy.Subscriber('/kinectv2/color/image_raw', Image, self.rgb_callback)
        self.camera_info_sub = rospy.Subscriber('/kinectv2/color/camera_info', CameraInfo, self.camera_info_callback)
        self.depth_points_sub = rospy.Subscriber('/kinectv2/depth/points', PointCloud2, self.depth_points_callback)

        # Publisher
        self.point_pub = rospy.Publisher('/kinect/object_coordinates', PointStamped, queue_size=10)

        # Camera related variables
        self.camera_info = None
        self.rgb_image = None
        self.depth_points = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Variables related to YOLO
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
        self.detectNames = ["bottle", "cup", "sports ball", "person"]

        # For frame rate (FPS)
        self.prev_time = 0
        self.fps = 0

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg)

    def depth_points_callback(self, msg):
        self.depth_points = msg

    def pixel_to_point(self, u, v):
        if self.depth_points is None:
            return None

        gen = point_cloud2.read_points(self.depth_points, field_names=("x", "y", "z"), skip_nans=True)
        while not rospy.is_shutdown():
            try:
                for point in gen:
                    depth_x = point[0]
                    depth_y = point[1]
                    depth_z = point[2]
                    if int(u) == int(depth_x) and int(v) == int(depth_y):
                        return depth_x, depth_y, depth_z
            except:
                return None

    def detect_objects(self, img):
        if self.rgb_image is None:
            return

        height, width, _ = self.rgb_image.shape

        results = YOLO("yolov8n.pt")(img, stream=True)

        for r in results:
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

                    if confidence >= 0.5:
                        # Draw bounding box around the detected object
                        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                        # Object details
                        org = [x1, y1]
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        fontScale = 1
                        color = (255, 0, 0)
                        thickness = 2

                        # Display class name
                        cv2.putText(img, cls, org, font, fontScale, color, thickness)

                        # Depth -> center_x and center_y are in pixel values (center of the bounding box)
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2

                        # Calculate world coordinates
                        x, y, z = self.pixel_to_point(center_x, center_y)

                        if x is not None and y is not None and z is not None:
                            # Publish the coordinates as a PointStamped message
                            point_msg = PointStamped()
                            point_msg.header.stamp = rospy.Time.now()
                            point_msg.header.frame_id = "kinect_frame"
                            point_msg.point.x = round(x, 3)
                            point_msg.point.y = round(y, 3)
                            point_msg.point.z = round(z, 3)
                            self.point_pub.publish(point_msg)

                            # Display coordinates
                            coord_text = f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}"
                            cv2.putText(img, coord_text, (org[0], org[1] + 30), font, fontScale, color, thickness)

                            print(f"Object: {cls}, Coordinates: ({x:.2f}, {y:.2f}, {z:.2f})")

        return img

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.rgb_image is not None:
                img = self.rgb_image.copy()
                img_with_boxes = self.detect_objects(img)

                # Calculate FPS
                current_time = time.time()
                self.fps = 1 / (current_time - self.prev_time)
                self.prev_time = current_time

                # Display FPS in the top right corner
                cv2.putText(img_with_boxes, f"FPS: {int(self.fps)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                            cv2.LINE_AA)

                cv2.imshow('RGB Image with Bounding Boxes', img_with_boxes)
                if cv2.waitKey(1) == ord('q'):
                    break

            rate.sleep()

def main():
    rospy.init_node('kinect_depth_processor')
    depth_processor = KinectDepthProcessor()
    depth_processor.run()

if __name__ == '__main__':
    main()
