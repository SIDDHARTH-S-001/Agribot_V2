#!/usr/bin/env python

import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from pynput import keyboard

class ArmControl:
    def __init__(self):
        rospy.init_node('arm_keyboard_control')
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.move_group = MoveGroupCommander("arm")
        self.target_pose = None
        self.step_size = 0.01  # Change this value as needed
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def set_target_pose(self, pose):
        self.target_pose = pose

    def increment_pose(self, axis, direction):
        if not self.target_pose:
            rospy.logerr("Target pose is not set!")
            return
        if axis == 'x':
            self.target_pose.pose.position.x += direction * self.step_size
        elif axis == 'y':
            self.target_pose.pose.position.y += direction * self.step_size
        elif axis == 'z':
            self.target_pose.pose.position.z += direction * self.step_size
        else:
            rospy.logerr("Invalid axis specified!")

    def on_press(self, key):
        try:
            if key.char == 'i':
                self.increment_pose('x', 1)
            elif key.char == 'd':
                self.increment_pose('x', -1)
            elif key.char == 'j':
                self.increment_pose('y', 1)
            elif key.char == 'k':
                self.increment_pose('y', -1)
            elif key.char == 'u':
                self.increment_pose('z', 1)
            elif key.char == 'm':
                self.increment_pose('z', -1)
        except AttributeError:
            pass

    def listen_keyboard(self):
        rospy.loginfo("Press 'i' to increment and 'd' to decrement the target pose in x, y, or z axis")
        self.listener.join()

if __name__ == '__main__':
    arm_control = ArmControl()
    initial_pose = arm_control.move_group.get_current_pose().pose
    arm_control.set_target_pose(PoseStamped(pose=initial_pose))
    arm_control.listen_keyboard()
