#ifndef GOAL_PUBLISHER_H_
#define GOAL_PUBLISHER_H_

#include <iostream>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class GoalPublisher
{
	public:
		GoalPublisher(const std::string & moveBaseTopic,
					  const std::string & basePoseTopic,
					  const std::string & controllerTopic);

		virtual ~GoalPublisher() = default;

		virtual int mainLoop() = 0;

		virtual void initializePose() = 0;

		// SET functions
		void setGoal(const Eigen::Vector3f & lookAtPos, const Eigen::Vector3f & goalPos, const tf::Transform & transform);

		static const int PROCESSING, REPLANING, SUCCEEDED;

	protected:
		void publishMoveBaseGoal();

		void sendMoveBaseGoal();

		virtual void computeMoveBaseGoal() = 0;

		void basePoseCallback(const nav_msgs::Odometry::ConstPtr & odomMsg);

		ros::NodeHandle nh;
		
		Eigen::Vector3f lookAtPosition;
		Eigen::Vector3f goalPosition;
		tf::Transform transformToMapFrame;

		geometry_msgs::PoseStamped moveBaseGoalPose;
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseActionClient;

		ros::Subscriber basePoseSubscriber;
		Eigen::Matrix4f basePose;

		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> controllerActionClient;

		int state = NO_GOAL;

		static const int NO_GOAL = 0;
		static const int READY = 1;
		static const int APPROACHING = 2;
		static const int APPROACHED = 3;
};

#endif /* GOAL_PUBLISHER_H_ */
