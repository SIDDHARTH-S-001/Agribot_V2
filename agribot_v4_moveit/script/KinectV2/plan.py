#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys

if __name__ == "__main__":
    rospy.init_node('move_group_interface_tutorial', anonymous=True)

    # MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    # the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    # are used interchangeably.
    PLANNING_GROUP_ARM = "arm_group"
    PLANNING_GROUP_GRIPPER = "gripper_group"

    # Initialize MoveItCommander
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate MoveGroupCommander objects
    move_group_interface_arm = moveit_commander.MoveGroupCommander(PLANNING_GROUP_ARM)
    move_group_interface_gripper = moveit_commander.MoveGroupCommander(PLANNING_GROUP_GRIPPER)

    # We can get a list of all the groups in the robot:
    rospy.loginfo("Available Planning Groups:")
    rospy.loginfo(move_group_interface_arm.get_active_joints())

    # 1. Move to home position
    move_group_interface_arm.set_named_target("default")

    my_plan_arm, planning_result = move_group_interface_arm.plan()

    # Check if planning was successful
    success = (planning_result.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)


    rospy.loginfo("Visualizing plan 1 (pose goal) %s", "SUCCESS" if success else "FAILED")

    move_group_interface_arm.execute(my_plan_arm)

    # 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
    current_pose = move_group_interface_arm.get_current_pose("ee_link").pose

    target_pose1 = geometry_msgs.msg.Pose()
    target_pose1.orientation = current_pose.orientation
    target_pose1.position.x = -3.5
    target_pose1.position.y = 0.4
    target_pose1.position.z = 0.528
    move_group_interface_arm.set_pose_target(target_pose1)

    my_plan_arm = move_group_interface_arm.plan()
    success = (my_plan_arm.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

    rospy.loginfo("Visualizing plan 2 (pose goal) %s", "SUCCESS" if success else "FAILED")

    move_group_interface_arm.execute(my_plan_arm)

    # 3. Open the gripper
    move_group_interface_gripper.set_named_target("open")
    move_group_interface_gripper.go()

    # 4. Move the TCP close to the object
    target_pose1.position.z -= 0.2
    move_group_interface_arm.set_pose_target(target_pose1)

    my_plan_arm = move_group_interface_arm.plan()
    success = (my_plan_arm.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

    rospy.loginfo("Visualizing plan 3 (pose goal) %s", "SUCCESS" if success else "FAILED")

    move_group_interface_arm.execute(my_plan_arm)

    # 5. Close the gripper
    move_group_interface_gripper.set_named_target("closed")
    move_group_interface_gripper.go()

    # 6. Move the TCP above the plate
    target_pose1.position.z += 0.2
    target_pose1.position.x -= 0.6
    move_group_interface_arm.set_pose_target(target_pose1)

    my_plan_arm = move_group_interface_arm.plan()
    success = (my_plan_arm.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

    rospy.loginfo("Visualizing plan 4 (pose goal) %s", "SUCCESS" if success else "FAILED")

    move_group_interface_arm.execute(my_plan_arm)

    # 7. Lower the TCP above the plate
    target_pose1.position.z -= 0.14
    move_group_interface_arm.set_pose_target(target_pose1)

    my_plan_arm = move_group_interface_arm.plan()
    success = (my_plan_arm.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

    rospy.loginfo("Visualizing plan 5 (pose goal) %s", "SUCCESS" if success else "FAILED")

    move_group_interface_arm.execute(my_plan_arm)

    # 8. Open the gripper
    move_group_interface_gripper.set_named_target("open")
    move_group_interface_gripper.go()
