#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
import gazebo_msgs.msg
from tf import transformations
import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy

import math

fetchState = gazebo_msgs.msg.ModelState()
armJointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
armUpJointPositions = [0.0, -1.22, 0.0, 0.65, 0.0, 0.00, 0.0]
armDownJointPositions = [0.0, -1.22, 0.0, 0.65, 0.0, 0.40, 0.0]
armMiddleJointPositions = [0.0, -1.22, 0.0, 0.65, 0.0, 0.20, 0.0]


class AutoSpin(object):

    def __init__(self, pose, radius):
        self.__currPose = pose
        self.__swayNum = 0
        self.__moveNum = 0
        self.__circleNum = 2
        self.__moveAngle = 30.0 / 180 * math.pi
        self.__swayAngel = 10.0 / 180 * math.pi
        self.__radius = radius

    def setCurrPose(self, pose):
        self.__currPose = pose

    def getCurrPose(self):
        return self.__currPose

    def getNextPose(self):
        if self.__swayNum < 8:
            nextPose = geometry_msgs.msg.Pose()
            nextPose.position = self.__currPose.position
            quat = [self.__currPose.orientation.x,
                    self.__currPose.orientation.y,
                    self.__currPose.orientation.z,
                    self.__currPose.orientation.w]

            currEulerAngle = list(
                transformations.euler_from_quaternion(quat))

            if self.__swayNum < 2 or self.__swayNum >= 6:
                currEulerAngle[2] += self.__swayAngel
                quat = transformations.quaternion_from_euler(
                    *currEulerAngle)
            else:
                currEulerAngle[2] -= self.__swayAngel
                quat = transformations.quaternion_from_euler(
                    *currEulerAngle)

            nextPose.orientation.x = quat[0]
            nextPose.orientation.y = quat[1]
            nextPose.orientation.z = quat[2]
            nextPose.orientation.w = quat[3]
            self.__swayNum += 1

            return nextPose
        else:
            self.__swayNum = 0
            nextPose = geometry_msgs.msg.Pose()
            zAxis = (0, 0, 1)
            modelPoint = (self.__radius, 0, 0)
            rotMat = transformations.rotation_matrix(
                self.__moveAngle, zAxis, modelPoint)

            currPose = numpy.array([
                [self.__currPose.position.x],
                [self.__currPose.position.y],
                [self.__currPose.position.z],
                [1]])
            nextP = numpy.dot(rotMat, currPose)
            nextPose.position.x = nextP[0][0]
            nextPose.position.y = nextP[1][0]
            nextPose.position.z = nextP[2][0]

            quat = [self.__currPose.orientation.x,
                    self.__currPose.orientation.y,
                    self.__currPose.orientation.z,
                    self.__currPose.orientation.w]

            currEulerAngle = list(
                transformations.euler_from_quaternion(quat))
            currEulerAngle[2] += self.__moveAngle
            quat = transformations.quaternion_from_euler(
                *currEulerAngle)

            nextPose.orientation.x = quat[0]
            nextPose.orientation.y = quat[1]
            nextPose.orientation.z = quat[2]
            nextPose.orientation.w = quat[3]
            self.__moveNum += 1
            return nextPose

    def hasNextPose(self):
        if self.__moveNum < 2 * math.pi / self.__moveAngle * self.__circleNum:
            return True
        else:
            return False


def statesCallback(data):
    for index, name in enumerate(data.name):
        if name == 'fetch':
            fetchState.model_name = 'fetch'
            fetchState.pose = data.pose[index]
            fetchState.twist = data.twist[index]
            fetchState.reference_frame = 'world'


def tiltWristGoal(beginPositions, endPositions):
    trajectory = JointTrajectory()
    trajectory.joint_names = armJointNames
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = beginPositions
    trajectory.points[0].velocities =  [0.0] * len(beginPositions)
    trajectory.points[0].accelerations = [0.0] * len(beginPositions)
    trajectory.points[0].time_from_start = rospy.Duration(0.0)
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[1].positions = endPositions
    trajectory.points[1].velocities =  [0.0] * len(endPositions)
    trajectory.points[1].accelerations = [0.0] * len(endPositions)
    trajectory.points[1].time_from_start = rospy.Duration(2.0)

    armGoal = FollowJointTrajectoryGoal()
    armGoal.trajectory = trajectory
    armGoal.goal_time_tolerance = rospy.Duration(0.0)
    return armGoal


def main():
    rospy.init_node('autospin', anonymous=True)
    modelStatePub = rospy.Publisher(
        '/gazebo/set_model_state',
        gazebo_msgs.msg.ModelState,
        queue_size=10)
    rospy.Subscriber(
        '/gazebo/model_states',
        gazebo_msgs.msg.ModelStates,
        statesCallback)

    while fetchState.model_name != 'fetch':
        rospy.sleep(3.)

    rospy.loginfo("Waiting for arm_controller...")
    armClient = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    armClient.wait_for_server()
    rospy.loginfo("...connected.")

    autoSpin = AutoSpin(fetchState.pose, 2.5)
    print "Initialization finished!"

    while not rospy.is_shutdown():
        if autoSpin.hasNextPose():
            pubData = gazebo_msgs.msg.ModelState()
            pubData.model_name = 'fetch'
            pubData.pose = autoSpin.getNextPose()
            pubData.twist.linear.x = 0
            pubData.twist.linear.y = 0
            pubData.twist.linear.z = 0
            pubData.twist.angular.x = 0
            pubData.twist.angular.y = 0
            pubData.twist.angular.z = 0
            pubData.reference_frame = 'world'

            modelStatePub.publish(pubData)
            print "New pose published!"
            print pubData
        else:
            pass
        armClient.send_goal(tiltWristGoal(armMiddleJointPositions, armUpJointPositions))
        armClient.wait_for_result(rospy.Duration(2.0))
        armClient.send_goal(tiltWristGoal(armUpJointPositions, armDownJointPositions))
        armClient.wait_for_result(rospy.Duration(4.0))
        armClient.send_goal(tiltWristGoal(armDownJointPositions, armMiddleJointPositions))
        armClient.wait_for_result(rospy.Duration(2.0))
        #rospy.sleep(2.)
        autoSpin.setCurrPose(fetchState.pose)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
