#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class WheelVelocityConverter:
    def __init__(self, wheel_radius, base_width):
        self.wheel_radius = wheel_radius
        self.base_width = base_width

        self.right_wheel_pub = rospy.Publisher('/right_wheel_vel', Float64, queue_size=10)
        self.left_wheel_pub = rospy.Publisher('/left_wheel_vel', Float64, queue_size=10)

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, twist_msg):
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z

        # Calculate wheel velocities using inverse kinematics
        right_wheel_velocity = ((2 * linear_velocity) + (self.base_width * angular_velocity)) / (2 * self.wheel_radius)
        left_wheel_velocity = ((2 * linear_velocity) - (self.base_width * angular_velocity)) / (2 * self.wheel_radius)

        print(right_wheel_velocity, left_wheel_velocity)

        # Publish wheel velocities
        self.right_wheel_pub.publish(round(right_wheel_velocity, 2))
        self.left_wheel_pub.publish(round(left_wheel_velocity, 2))

if __name__ == '__main__':
    rospy.init_node('wheel_velocity_converter', anonymous=True)

    wheel_radius = 0.2032  # Replace with your wheel radius in meters
    base_width = 0.481224   # Replace with your robot's base width in meters

    converter = WheelVelocityConverter(wheel_radius, base_width)

    rospy.spin()
