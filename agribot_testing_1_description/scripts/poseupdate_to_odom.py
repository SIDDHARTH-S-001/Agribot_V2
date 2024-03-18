#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Pose, Point
from nav_msgs.msg import Odometry
import tf.transformations as tf


class PoseUpdateToOdom:
    def __init__(self):
        rospy.init_node('poseupdate_to_odom', anonymous=True)
        self.pose_sub = rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, self.pose_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    def pose_callback(self, pose_msg):
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = "odom_1"

        # Extract position and orientation from pose
        position = pose_msg.pose.pose.position
        orientation = pose_msg.pose.pose.orientation

        # Convert orientation to yaw angle
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf.euler_from_quaternion(orientation_list)

        # Populate odometry message
        odom_msg.pose.pose.position = Point(position.x, position.y, position.z)
        odom_msg.pose.pose.orientation = Quaternion(*tf.quaternion_from_euler(0, 0, yaw))
        odom_msg.pose.covariance = pose_msg.pose.covariance

        # Publish odometry message
        self.odom_pub.publish(odom_msg)
        print(odom_msg)


if __name__ == '__main__':
    try:
        pose_update_to_odom = PoseUpdateToOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
