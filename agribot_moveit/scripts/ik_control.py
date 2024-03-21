#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import sys
import tty
import termios

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.current_pose = self.move_group.get_current_pose().pose
        self.rate = rospy.Rate(10)  # 10 Hz
        self.setup_keyboard()

    def setup_keyboard(self):
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def get_key(self):
        try:
            key = sys.stdin.read(1)
            return key
        except IOError:
            return ''

    def run(self):
        print("Use keyboard keys to control the robot's target pose:")
        print("q: Increment x, a: Decrement x")
        print("w: Increment y, s: Decrement y")
        print("e: Increment z, d: Decrement z")
        print("Press Ctrl+C to exit")

        while not rospy.is_shutdown():
            key = self.get_key()
            if key == 'q':
                self.current_pose.position.x += 0.05
            elif key == 'a':
                self.current_pose.position.x -= 0.05
            elif key == 'w':
                self.current_pose.position.y += 0.05
            elif key == 's':
                self.current_pose.position.y -= 0.05
            elif key == 'e':
                self.current_pose.position.z += 0.05
            elif key == 'd':
                self.current_pose.position.z -= 0.05
            elif key == '\x03':  # Ctrl+C
                break
            
            # Set the new target pose
            self.move_group.set_pose_target(self.current_pose)
            self.move_group.go(wait=True)
            self.rate.sleep()

    def restore_terminal_settings(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def __del__(self):
        self.restore_terminal_settings()

if __name__ == '__main__':
    try:
        controller = KeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
