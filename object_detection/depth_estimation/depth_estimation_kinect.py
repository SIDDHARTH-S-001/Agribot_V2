import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class KinectDepthProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/your/depth/topic', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/your/rgb/camera_info/topic', CameraInfo, self.camera_info_callback)
        self.camera_info = None
        self.depth_image = None
        self.camera_matrix = None
        self.distortion_matrix = None

    def load_camera_parameters(self, camera_matrix_file, distortion_matrix_file):
        # Load camera matrix and distortion coefficients from text files
        self.camera_matrix = np.loadtxt(camera_matrix_file)
        self.distortion_matrix = np.loadtxt(distortion_matrix_file)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    def pixel_to_point(self, u, v, depth):
        if self.camera_matrix is None:
            return None

        if depth == 0:
            return None

        inv_camera_matrix = np.linalg.inv(self.camera_matrix)
        uv_1 = np.array([u, v, 1])
        point_3d = np.dot(inv_camera_matrix, uv_1) * depth

        return point_3d

    def process_depth_image(self):
        if self.depth_image is None or self.camera_matrix is None:
            return

        height, width = self.depth_image.shape[:2]
        for v in range(height):
            for u in range(width):
                depth = self.depth_image[v, u]
                point = self.pixel_to_point(u, v, depth)
                if point is not None:
                    print("Pixel ({}, {}): Depth = {}, Point = {}".format(u, v, depth, point))


def main():
    rospy.init_node('kinect_depth_processor')
    depth_processor = KinectDepthProcessor()

    # Specify the paths to camera matrix and distortion matrix text files
    camera_matrix_file = 'path/to/camera_matrix.txt'
    distortion_matrix_file = 'path/to/distortion_matrix.txt'
    depth_processor.load_camera_parameters(camera_matrix_file, distortion_matrix_file)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        depth_processor.process_depth_image()
        rate.sleep()

if __name__ == '__main__':
    main()
