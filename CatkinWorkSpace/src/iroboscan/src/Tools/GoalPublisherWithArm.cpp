#include "GoalPublisherWithArm.h"

GoalPublisherWithArm::GoalPublisherWithArm(const std::string & moveBaseTopic,
										   const std::string & basePoseTopic,
										   const std::string & controllerTopic,
							 			   const std::string & groupName)
 : GoalPublisher(moveBaseTopic, basePoseTopic, controllerTopic),
   moveGroup(groupName)
{
	// Filter zero joint values
	moveGroup.getCurrentJointValues();

	setInitialMoveitGoalPose();
}

int GoalPublisherWithArm::mainLoop()
{
	switch (state)
	{
		case READY:
			initializePose();
			publishMoveBaseGoal();
			tiltWrist(paraMinWalkingWristFlex, paraMaxWalkingWristFlex);
			state = APPROACHING;
			break;
		case APPROACHING:
			if (controllerActionClient.getState() != actionlib::SimpleClientGoalState::ACTIVE)
				tiltWrist(paraMinWalkingWristFlex, paraMaxWalkingWristFlex);

			if (moveBaseActionClient.getState() != actionlib::SimpleClientGoalState::ACTIVE)
				std::cout << "Goal State: " << moveBaseActionClient.getState().toString() << std::endl;

			if (moveBaseActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				arrivedUp = false;
				arrivedDown = false;
				state = LOOKING_UP_DOWN;
			}
			else if (moveBaseActionClient.getState() == actionlib::SimpleClientGoalState::ABORTED)
			{
				state = NO_GOAL;
				return REPLANING;
			}

			break;
		case LOOKING_UP_DOWN:
			if (controllerActionClient.getState() != actionlib::SimpleClientGoalState::ACTIVE)
			{
				if (tiltWrist(paraMinLookingWristFlex, paraMaxLookingWristFlex))
				{
					arrivedRight = false;
					arrivedLeft = false;
					state = LOOKING_LEFT_RIGHT;
				}
			}

			break;
		case LOOKING_LEFT_RIGHT:
			if (controllerActionClient.getState() != actionlib::SimpleClientGoalState::ACTIVE)
			{
				if (rotateShoulder(paraMinLookingShoulderPan, paraMaxLookingShoulderPan))
				{
					beginPausingTime = std::chrono::system_clock::now();
					state = APPROACHED;
				}
			}
			
			break;
		case APPROACHED:
			{
				auto pauseDuration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - beginPausingTime);
				if (pauseDuration.count() > 2)
				{
					publishMoveitGoal();
					state = NO_GOAL;
					return SUCCEEDED;
				}
			}
			break;
		case NO_GOAL:
		default:
			break;
	}
	return PROCESSING;
}

void GoalPublisherWithArm::publishMoveitGoal()
{
	computeMoveitGoal();
	sendMoveitGoal();
}

void GoalPublisherWithArm::computeMoveBaseGoal()
{
	tf::Vector3 tfLookAtPos(lookAtPosition[0], lookAtPosition[1], lookAtPosition[2]);
	tf::Vector3 tfGoalPos(goalPosition[0], goalPosition[1], goalPosition[2]);

	tfLookAtPos = transformToMapFrame * tfLookAtPos;
	tfGoalPos = transformToMapFrame * tfGoalPos;

	Eigen::Vector3f mapLookAtPos(tfLookAtPos.x(), tfLookAtPos.y(), 0.0f);
	Eigen::Vector3f mapGoalPos(tfGoalPos.x(), tfGoalPos.y(), 0.0f);

	Eigen::Vector3f mapUpVector(0, 0, 1);
	Eigen::Vector3f nbvXAxis = (mapLookAtPos - mapGoalPos).normalized();
	Eigen::Vector3f nbvYAxis = mapUpVector.cross(nbvXAxis).normalized();
	Eigen::Vector3f nbvZAxis = nbvXAxis.cross(nbvYAxis).normalized();

	Eigen::Matrix3f mapRotMat;
	mapRotMat.col(0) = nbvXAxis;
	mapRotMat.col(1) = nbvYAxis;
	mapRotMat.col(2) = nbvZAxis;

	Eigen::Quaternionf mapQuat(mapRotMat);

	mapGoalPos -= 0.65 * nbvXAxis;

	moveBaseGoalPose.header.stamp = ros::Time();
	moveBaseGoalPose.pose.position.x = mapGoalPos.x();
	moveBaseGoalPose.pose.position.y = mapGoalPos.y();
	moveBaseGoalPose.pose.position.z = mapGoalPos.z();
	std::cout << "MoveBase goal position: " << mapGoalPos.x() << " "
									   << mapGoalPos.y() << " "
									   << mapGoalPos.z() << std::endl;

	moveBaseGoalPose.pose.orientation.x = mapQuat.x();
	moveBaseGoalPose.pose.orientation.y = mapQuat.y();
	moveBaseGoalPose.pose.orientation.z = mapQuat.z();
	moveBaseGoalPose.pose.orientation.w = mapQuat.w();
	std::cout << "MoveBase goal orientation: " << mapQuat.x() << " "
										  << mapQuat.y() << " "
										  << mapQuat.z() << " "
										  << mapQuat.w() << std::endl;
}

void GoalPublisherWithArm::computeMoveitGoal()
{
	tf::Vector3 tfLookAtPos(lookAtPosition[0], lookAtPosition[1], lookAtPosition[2]);
	tf::Vector3 tfGoalPos(goalPosition[0], goalPosition[1], goalPosition[2]);

	tfLookAtPos = transformToMapFrame * tfLookAtPos;
	tfGoalPos = transformToMapFrame * tfGoalPos;

	Eigen::Vector3f mapLookAtPos(tfLookAtPos.x(), tfLookAtPos.y(), tfLookAtPos.z());
	Eigen::Vector3f mapGoalPos(tfGoalPos.x(), tfGoalPos.y(), tfGoalPos.z());

	Eigen::Vector3f mapUpVector(0, 0, 1);
	Eigen::Vector3f nbvXAxis = (mapLookAtPos - mapGoalPos).normalized();
	Eigen::Vector3f nbvYAxis = mapUpVector.cross(nbvXAxis).normalized();
	Eigen::Vector3f nbvZAxis = nbvXAxis.cross(nbvYAxis).normalized();

	Eigen::Matrix3f mapRotMat;
	mapRotMat.col(0) = nbvXAxis;
	mapRotMat.col(1) = nbvYAxis;
	mapRotMat.col(2) = nbvZAxis;

	mapGoalPos -= 0.22145 * nbvXAxis;
	mapGoalPos -= 0.02 * nbvYAxis;
	mapGoalPos -= 0.0225 * nbvZAxis;

	Eigen::Matrix4f mapGoalPose;
	mapGoalPose.setIdentity();

	mapGoalPose.topLeftCorner(3, 3) = mapRotMat;
	mapGoalPose.topRightCorner(3, 1) = mapGoalPos;

	Eigen::Matrix4f relativeGoalPose = basePose.inverse() * mapGoalPose;

	moveitGoalPose.position.x = relativeGoalPose(0, 3);
	moveitGoalPose.position.y = relativeGoalPose(1, 3);
	moveitGoalPose.position.z = relativeGoalPose(2, 3);

	Eigen::Quaternionf relativeQuat(Eigen::Matrix3f(relativeGoalPose.topLeftCorner(3, 3)));
	moveitGoalPose.orientation.x = relativeQuat.x();
	moveitGoalPose.orientation.y = relativeQuat.y();
	moveitGoalPose.orientation.z = relativeQuat.z();
	moveitGoalPose.orientation.w = relativeQuat.w();

	std::cout << "Moveit goal position: " << relativeGoalPose(0, 3) << " "
										  << relativeGoalPose(1, 3) << " "
										  << relativeGoalPose(2, 3) << std::endl;

	std::cout << "Moveit goal orientation: " << relativeQuat.x() << " "
											 << relativeQuat.y() << " "
											 << relativeQuat.z() << " "
											 << relativeQuat.w() << std::endl;
}

void GoalPublisherWithArm::initializePose()
{
	ros::AsyncSpinner asyncSpinner(1);
	asyncSpinner.start();

	moveGroup.setJointValueTarget(initialMoveitGoalPose);
	moveit::planning_interface::MoveGroup::Plan moveGroupPlan;
	bool planSuccess = moveGroup.plan(moveGroupPlan);
	if (planSuccess)
	{
		moveGroup.execute(moveGroupPlan);
	}
	else
	{
		std::cerr << "Error: could not find a valid move group plan!" << std::endl;
	}
	asyncSpinner.stop();
}

bool GoalPublisherWithArm::tiltWrist(double minWristFlex, double maxWristFlex)
{
	std::vector<double> currentJointPositions = moveGroup.getCurrentJointValues();
	std::vector<double> zeroVec(currentJointPositions.size(), 0);
	if (currentJointPositions == zeroVec)
		return false;

	control_msgs::FollowJointTrajectoryGoal tiltWristGoal;
	trajectory_msgs::JointTrajectory trajectory;
	trajectory.points.resize(2);
	trajectory.joint_names = moveGroup.getJointNames();

	if (currentJointPositions[6] >= maxWristFlex)
	{
		tiltStep = -0.1;
		arrivedDown = true;
	}
	else if (currentJointPositions[6] <= minWristFlex)
	{
		arrivedUp = true;
		tiltStep = 0.1;
	}

	trajectory.points[0].positions = currentJointPositions;
	trajectory.points[0].velocities = zeroVec;
	trajectory.points[0].accelerations = zeroVec;
	trajectory.points[0].time_from_start = ros::Duration(0.0);

	currentJointPositions[6] += tiltStep;
	trajectory.points[1].positions = currentJointPositions;
	trajectory.points[1].velocities = zeroVec;
	trajectory.points[1].accelerations = zeroVec;
	trajectory.points[1].time_from_start = ros::Duration(0.5);

	tiltWristGoal.trajectory = trajectory;
	tiltWristGoal.goal_time_tolerance = ros::Duration(0.0);

	controllerActionClient.sendGoal(tiltWristGoal);

	if (arrivedUp && arrivedDown && std::abs(currentJointPositions[6] - (minWristFlex + maxWristFlex) / 2) <= 0.2)
		return true;
	else
		return false;
}

bool GoalPublisherWithArm::rotateShoulder(double minShoulderPan, double maxShoulderPan)
{
	std::vector<double> currentJointPositions = moveGroup.getCurrentJointValues();
	std::vector<double> zeroVec(currentJointPositions.size(), 0);
	if (currentJointPositions == zeroVec)
		return false;

	control_msgs::FollowJointTrajectoryGoal rotateShoulderGoal;
	trajectory_msgs::JointTrajectory trajectory;
	trajectory.points.resize(2);
	trajectory.joint_names = moveGroup.getJointNames();

	if (currentJointPositions[1] >= maxShoulderPan)
	{
		rotateStep = -0.1;
		arrivedLeft = true;
	}
	else if (currentJointPositions[1] <= minShoulderPan)
	{
		rotateStep = 0.1;
		arrivedRight = true;
	}

	trajectory.points[0].positions = currentJointPositions;
	trajectory.points[0].velocities = zeroVec;
	trajectory.points[0].accelerations = zeroVec;
	trajectory.points[0].time_from_start = ros::Duration(0.0);

	currentJointPositions[1] += rotateStep;
	trajectory.points[1].positions = currentJointPositions;
	trajectory.points[1].velocities = zeroVec;
	trajectory.points[1].accelerations = zeroVec;
	trajectory.points[1].time_from_start = ros::Duration(0.5);

	rotateShoulderGoal.trajectory = trajectory;
	rotateShoulderGoal.goal_time_tolerance = ros::Duration(0.0);

	controllerActionClient.sendGoal(rotateShoulderGoal);
	
	if (arrivedLeft && arrivedRight && std::abs(currentJointPositions[1] - (minShoulderPan + maxShoulderPan) / 2) <= 0.2)
		return true;
	else
		return false;
}

void GoalPublisherWithArm::sendMoveitGoal()
{
	ros::AsyncSpinner asyncSpinner(1);
	asyncSpinner.start();

	moveGroup.setPoseTarget(moveitGoalPose);
	moveit::planning_interface::MoveGroup::Plan moveGroupPlan;
	bool planSuccess = moveGroup.plan(moveGroupPlan);
	if (planSuccess)
	{
		moveGroup.execute(moveGroupPlan);
	}
	else
	{
		std::cerr << "Error: could not find a valid move group plan!" << std::endl;
	}
	asyncSpinner.stop();
}

void GoalPublisherWithArm::setInitialMoveitGoalPose()
{
	// torso_lift_joint
	initialMoveitGoalPose.push_back(0.0);

	// shoulder_pan_joint
	initialMoveitGoalPose.push_back(0.0);

	// shoulder_lift_joint
	initialMoveitGoalPose.push_back(-1.22);

	// upperarm_roll_joint
	initialMoveitGoalPose.push_back(0.0);

	// elbow_flex_joint
	initialMoveitGoalPose.push_back(0.65);

	// forearm_roll_joint
	initialMoveitGoalPose.push_back(0.0);

	// wrist_flex_joint
	initialMoveitGoalPose.push_back(1.05); //1.12

	// wrist_roll_joint
	initialMoveitGoalPose.push_back(0.0);

	//constexpr double pi = 3.1415926;

	//initialMoveitGoalPose.position.x = 0.4; // 0.4
	//initialMoveitGoalPose.position.y = 0.0;
	//initialMoveitGoalPose.position.z = 1.0; // 1.0

	//initialMoveitGoalPose.orientation.x = 0.0;
	//initialMoveitGoalPose.orientation.y = std::sin(0.5 * pi / 6);
	//initialMoveitGoalPose.orientation.z = 0.0;
	//initialMoveitGoalPose.orientation.w = std::cos(0.5 * pi / 6);
}
