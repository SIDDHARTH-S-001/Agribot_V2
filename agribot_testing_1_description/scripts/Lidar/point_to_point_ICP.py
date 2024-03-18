#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.point_cloud2 import read_points
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sklearn.neighbors import NearestNeighbors
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs

class LidarICP:
    def __init__(self):
        rospy.init_node('lidar_icp_node', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pose_pub = rospy.Publisher('/updated_pose', PoseStamped, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.prev_scan = None

        # Broadcast initial static transform
        self.broadcast_initial_transform()

    def broadcast_initial_transform(self):
        initial_transform = TransformStamped()
        initial_transform.header.stamp = rospy.Time.now()
        initial_transform.header.frame_id = 'odom'
        initial_transform.child_frame_id = 'base_link'
        initial_transform.transform.translation.x = 0.0
        initial_transform.transform.translation.y = 0.0
        initial_transform.transform.translation.z = 0.0
        initial_transform.transform.rotation.x = 0.0
        initial_transform.transform.rotation.y = 0.0
        initial_transform.transform.rotation.z = 0.0
        initial_transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(initial_transform)

    def scan_callback(self, scan_msg):
        current_scan = self.laser_scan_to_point_cloud(scan_msg) # current_scan is an array of points
        
        if self.prev_scan is not None:
            # Apply ICP
            R, T = self.point_to_point_icp(self.prev_scan, current_scan)
            
            # Update pose and broadcast the dynamic transform
            self.update_pose_and_broadcast_transform(R, T)
        
        # Save the current scan for the next iteration
        self.prev_scan = current_scan

    def laser_scan_to_point_cloud(self, scan_msg):
        # Extract points from LaserScan message
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)

        # Convert polar coordinates to Cartesian coordinates
        points = []
        for i in range(len(ranges)):
            if ranges[i] < scan_msg.range_max:
                x = ranges[i] * np.cos(angles[i])
                y = ranges[i] * np.sin(angles[i])
                points.append([x, y])

        return np.array(points)

    def point_to_point_icp(self, source, target):
        # Use Nearest Neighbors to find correspondences
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(target)
        distances, indices = nbrs.kneighbors(source)

        # Extract matched points
        matched_target = target[indices.flatten()]
        
        # Compute the transformation
        R, T = self.calculate_icp_transformation(source, matched_target) # prev_scan is the source and current_scan is target

        return R, T

    def calculate_icp_transformation(self, source, target):
        # Calculate the transformation between source and target using SVD
        # prev_scan is the source and current_scan is target
        mean_source = np.mean(source, axis=0)
        mean_target = np.mean(target, axis=0)

        centered_source = source - mean_source
        centered_target = target - mean_target

        W = np.dot(centered_source.T, centered_target)

        U, _, Vt = np.linalg.svd(W)

        R = np.dot(Vt.T, U.T)
        T = mean_target - np.dot(R, mean_source)

        return R, T

    def update_pose_and_broadcast_transform(self, R, T):
        try:
            # Get the current robot pose from the TF tree
            trans = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation.x = trans.transform.rotation.x
            pose.pose.orientation.y = trans.transform.rotation.y
            pose.pose.orientation.z = trans.transform.rotation.z
            pose.pose.orientation.w = trans.transform.rotation.w

            # Update the pose with the ICP transformation
            updated_pose = self.apply_icp_transformation(pose, R, T)

            # Publish the updated pose
            self.pose_pub.publish(updated_pose)

            # Broadcast the dynamic transform between odom and base_link
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = 'odom'
            transform_stamped.child_frame_id = 'base_link'
            transform_stamped.transform.translation.x = T[0]
            transform_stamped.transform.translation.y = T[1]
            transform_stamped.transform.translation.z = T[2]
            q = quaternion_from_euler(0, 0, np.arctan2(R[1, 0], R[0, 0]))
            transform_stamped.transform.rotation.x = q[0]
            transform_stamped.transform.rotation.y = q[1]
            transform_stamped.transform.rotation.z = q[2]
            transform_stamped.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(transform_stamped)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error looking up the transform. Skipping update.")

    def apply_icp_transformation(self, pose, R, T):
        # Convert orientation to euler angles
        _, _, yaw = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y,
                                           pose.pose.orientation.z, pose.pose.orientation.w])

        # Apply the ICP transformation
        updated_pose = PoseStamped()
        updated_pose.header.frame_id = "odom"
        updated_pose.header.stamp = rospy.Time.now()
        updated_pose.pose.position.x = pose.pose.position.x + T[0]
        updated_pose.pose.position.y = pose.pose.position.y + T[1]
        updated_pose.pose.position.z = pose.pose.position.z + T[2]
        updated_yaw = yaw + np.arctan2(R[1, 0], R[0, 0])  # Extracting rotation from the transformation matrix
        q = quaternion_from_euler(0, 0, updated_yaw)
        updated_pose.pose.orientation.x = q[0]
        updated_pose.pose.orientation.y = q[1]
        updated_pose.pose.orientation.z = q[2]
        updated_pose.pose.orientation.w = q[3]

        return updated_pose

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    lidar_icp = LidarICP()
    lidar_icp.run()
