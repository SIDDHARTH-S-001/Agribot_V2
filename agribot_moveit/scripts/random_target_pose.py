#!/usr/bin/env python3
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
import sys

def move_to_target_pose():
    # Initialize MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_target_pose_node', anonymous=True)

    # Create a move group instance
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")
    end_effector_link = group.get_end_effector_link()
    # group.set_pose_reference_frame(end_effector_link)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

    # Get the current pose of the end-effector link
    # print("End-effector link: ", end_effector_link)
    # print(type(end_effector_link))
    # target_pose = PoseStamped()
    # group.set_planning_frame(end_effector_link)

    # while not rospy.is_shutdown():
    # #     # Define the target pose
    #     target_pose = PoseStamped()
    #     # target_pose = Pose()
    #     target_pose.header.frame_id = robot.get_planning_frame()
    #     target_pose.header.frame_id = end_effector_link
    #     print('q: x+0.05   |   w:x-0.05')
    #     print('a: y+0.05   |   s:y-0.05')
    #     print('z: z+0.05   |   x:z-0.05')
    #     key = input("Enter a command: ")
    #     if key == 'q':
    #         target_pose.pose.position.x +=0.05
    #     elif key == 'w':
    #         target_pose.pose.position.x +=0.05
    #     elif key == 'a':
    #         target_pose.pose.position.y +=0.05
    #     elif key == 's':
    #         target_pose.pose.position.y -=0.05
    #     elif key == 'z':
    #         target_pose.pose.position.z +=0.05
    #     elif key == 'x':
    #         target_pose.pose.position.z -=0.05
    #     else:
    #         continue
        
    # target_pose.header.frame_id = robot.get_planning_frame()

    # joint_values = group.compute_ik([target_pose], max_trials=1000)

    # if joint_values is not None:
    #     print("Joint values: {}".format(joint_values))
    # else:
    #     print("Inverse kinematics failed")

    # target_pose.pose.position.x = 0.15
    # target_pose.pose.position.y = 0.25
    # target_pose.pose.position.z = 0.15
    # target_pose.pose.orientation.x = 0
    # target_pose.pose.orientation.y = 0
    # target_pose.pose.orientation.z = 0
    # target_pose.pose.orientation.w = 1.0
    # print(target_pose)
    
    # print(group.get_planning_frame())

    # Set the target pose for the end-effector link
    # group.set_named_target('default')

    # Plan and execute the motion
    # group.set_pose_target(target_pose, end_effector_link)
    
    group.set_random_target()
    group.plan()
    group.go(wait=True)
    # group.execute(target_pose, wait=True)
    
    # group.stop()
    # group.clear_pose_targets()        

    # current_pose = group.get_current_pose().pose
    # Clean up
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_to_target_pose()
    except rospy.ROSInterruptException:
        pass