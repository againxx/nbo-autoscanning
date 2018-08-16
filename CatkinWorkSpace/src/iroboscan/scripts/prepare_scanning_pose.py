#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

armJointNames = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
armJointPositions = [0.0 ,0.0, -1.22, 0.0, 0.65, 0.0, 1.05, 0.0]


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("prepare_scanning_pose", anonymous=True)

    moveGroup = moveit_commander.MoveGroupCommander("arm_with_torso")
    moveGroup.set_max_acceleration_scaling_factor(0.05)
    moveGroup.set_max_velocity_scaling_factor(0.05)

    moveGroup.clear_pose_targets()
    moveGroup.set_joint_value_target(armJointPositions)

    rospy.loginfo("Planning trajectory...")
    armPlan = moveGroup.plan()
    moveGroup.go(wait=True)
    rospy.loginfo("...done")
