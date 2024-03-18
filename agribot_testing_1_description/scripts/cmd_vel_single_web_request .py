#!/usr/bin/env python

import rospy
import requests  # Add this import for HTTP requests
from geometry_msgs.msg import Twist

class WheelVelocityConverter:
    def __init__(self, wheel_radius, base_width, server_url):
        self.wheel_radius = wheel_radius
        self.base_width = base_width
        self.server_url = server_url

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, twist_msg):
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z
        print(linear_velocity, angular_velocity)

        # Calculate wheel velocities using inverse kinematics
        right_wheel_velocity = ((2 * linear_velocity) - (self.base_width * angular_velocity)) / (2 * self.wheel_radius)
        left_wheel_velocity = ((2 * linear_velocity) + (self.base_width * angular_velocity)) / (2 * self.wheel_radius)
        print('calculations done')

        # Send HTTP requests with speed values
        self.send_speed_request('left', 'right',  left_wheel_velocity, right_wheel_velocity)
        # print('requests not sent')

        rospy.sleep(0.1)

    def send_speed_request(self, leftwheel, rightwheel, leftvelocity, rightvelocity):
        url = f"{self.server_url}/set_speed?leftwheel={leftwheel}&leftspeed={leftvelocity:.2f}&rightwheel={rightwheel}&rightspeed={rightvelocity:.2f}"
        print(url)
        try:
            response = requests.get(url)
            response.raise_for_status()  # Raise an exception for HTTP errors
            rospy.loginfo(f"Successfully sent speed request.")
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Failed to send request. Error: {e}")


if __name__ == '__main__':
    rospy.init_node('wheel_velocity_converter', anonymous=True, log_level=rospy.DEBUG)

    wheel_radius = 0.2032/2  # Replace with your wheel radius in meters
    base_width = 0.481224   # Replace with your robot's base width in meters
    server_url = 'http://192.168.143.67'  # Replace with your server address

    converter = WheelVelocityConverter(wheel_radius, base_width, server_url)

    rospy.spin()
