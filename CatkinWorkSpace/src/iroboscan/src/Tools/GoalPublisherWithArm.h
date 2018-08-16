#ifndef GOAL_PUBLISHER_WITH_ARM_H_
#define GOAL_PUBLISHER_WITH_ARM_H_

#include <moveit/move_group_interface/move_group.h>

#include "GoalPublisher.h"

class GoalPublisherWithArm : public GoalPublisher
{
	public:
		GoalPublisherWithArm(const std::string & moveBaseTopic,
					  		 const std::string & basePoseTopic,
					  		 const std::string & controllerTopic,
					  		 const std::string & groupName);

		~GoalPublisherWithArm() = default;

		int mainLoop();

		void initializePose();

	private:
		void setInitialMoveitGoalPose();

		void publishMoveitGoal();

		void computeMoveBaseGoal();
		void computeMoveitGoal();

		void sendMoveitGoal();

		bool tiltWrist(double minWristFlex, double maxWristFlex);
		bool rotateShoulder(double minShoulderPan, double maxShoulderPan);

		//geometry_msgs::Pose initialMoveitGoalPose;
		std::vector<double> initialMoveitGoalPose;
		geometry_msgs::Pose moveitGoalPose;
		moveit::planning_interface::MoveGroup moveGroup;

		double tiltStep = 0.1;
		double rotateStep = 0.1;

		double paraMinWalkingWristFlex = 0.9;
		double paraMaxWalkingWristFlex = 1.3;

		double paraMinLookingWristFlex = 0.6;
		double paraMaxLookingWristFlex = 1.5;

		double paraMinLookingShoulderPan = -0.3;
		double paraMaxLookingShoulderPan = 0.3;

		bool arrivedUp = false;
		bool arrivedDown = false;
		bool arrivedRight = false;
		bool arrivedLeft = false;

		std::chrono::system_clock::time_point beginPausingTime;

		static const int LOOKING_UP_DOWN = 4;
		static const int LOOKING_LEFT_RIGHT = 5;
};

#endif /* GOAL_PUBLISHER_WITH_ARM_H_ */
