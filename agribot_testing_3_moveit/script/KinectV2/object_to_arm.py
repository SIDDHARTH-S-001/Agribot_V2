#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_matrix
import tf2_ros
import tf2_geometry_msgs

class KinectToArmFrameConverter:
    def __init__(self):
        rospy.init_node('kinect_to_arm_frame_converter')
        self.object_sub = rospy.Subscriber('/kinect/object_coordinates', PointStamped, self.object_callback)
        self.object_pub = rospy.Publisher('/arm/object_coordinates', PointStamped, queue_size=10)

        # Transformation matrix from Kinect to arm base
        self.kinect_to_arm_base_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # Need to modify this with actual transformation
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def object_callback(self, msg):
        try:
            # Lookup transformation from base_link to l4_frame
            trans = self.tfBuffer.lookup_transform('base_link', 'l4_frame', rospy.Time(), rospy.Duration(1.0))
            transform_matrix = quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            transform_matrix[:3, 3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            
            # Transformation from Kinect to arm base
            object_coordinates_kinect = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0])
            object_coordinates_arm_base = np.dot(self.kinect_to_arm_base_matrix, object_coordinates_kinect)

            # Transformation from arm base to l4_frame
            object_coordinates_l4_frame = np.dot(transform_matrix, object_coordinates_arm_base)

            # Publish the transformed coordinates
            transformed_msg = PointStamped()
            transformed_msg.header.stamp = rospy.Time.now()
            transformed_msg.header.frame_id = 'l4_frame'  # Modify frame ID to the last link of your arm
            transformed_msg.point.x = object_coordinates_l4_frame[0]
            transformed_msg.point.y = object_coordinates_l4_frame[1]
            transformed_msg.point.z = object_coordinates_l4_frame[2]
            self.object_pub.publish(transformed_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    converter = KinectToArmFrameConverter()
    converter.run()
