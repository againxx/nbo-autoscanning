#include "GoalPublisherWithHead.h"

GoalPublisherWithHead::GoalPublisherWithHead(const std::string & moveBaseTopic,
							 				 const std::string & basePoseTopic,
											 const std::string & controllerTopic)
 : GoalPublisher(moveBaseTopic, basePoseTopic, controllerTopic)
{

}

int GoalPublisherWithHead::mainLoop()
{
	switch (state)
	{
		case READY:
			publishMoveBaseGoal();
			state = APPROACHING;
			break;
		case APPROACHING:
			if (moveBaseActionClient.getState() != actionlib::SimpleClientGoalState::ACTIVE)
				std::cout << "Goal State: " << moveBaseActionClient.getState().toString() << std::endl;

			if (moveBaseActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
                arrivedUp = false;
                arrivedDown = false;
                controlHead(paraMiddleHeadTilt, paraUpHeadTilt, 1);
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
                if (!arrivedUp)
                {
                    arrivedUp = true;
                    controlHead(paraUpHeadTilt, paraDownHeadTilt, 1);
                }
                else if (!arrivedDown)
                {
                    arrivedDown = true;
                    controlHead(paraDownHeadTilt, paraMiddleHeadTilt, 1);
                }
                else
                {
                    arrivedLeft = false;
                    arrivedRight = false;
                    controlHead(paraMiddleHeadPan, paraLeftHeadPan, 0);
                    state = LOOKING_LEFT_RIGHT;
                }
			}
            break;
        case LOOKING_LEFT_RIGHT:
			if (controllerActionClient.getState() != actionlib::SimpleClientGoalState::ACTIVE)
			{
                if (!arrivedLeft)
                {
                    arrivedLeft = true;
                    controlHead(paraLeftHeadPan, paraRightHeadPan, 0);
                }
                else if (!arrivedRight)
                {
                    arrivedRight = true;
                    controlHead(paraRightHeadPan, paraMiddleHeadPan, 0);
                }
                else
                {
                    state = APPROACHED;
                }
			}
            break;
		case APPROACHED:
			{
				auto pauseDuration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - beginPausingTime);
				if (pauseDuration.count() > 5)
				{
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

void GoalPublisherWithHead::computeMoveBaseGoal()
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

void GoalPublisherWithHead::controlHead(double currentJointValue, double desiredJointValue, int jointIndex)
{
	control_msgs::FollowJointTrajectoryGoal headGoal;
	trajectory_msgs::JointTrajectory trajectory;
	trajectory.points.resize(2);
	trajectory.joint_names.resize(2);
	trajectory.joint_names[0] = "head_pan_joint";
	trajectory.joint_names[1] = "head_tilt_joint";

	std::vector<double> zeroVec(2, 0);

	trajectory.points[0].positions = zeroVec;
    trajectory.points[0].positions[1] = paraMiddleHeadTilt;
    trajectory.points[0].positions[jointIndex] = currentJointValue;
	trajectory.points[0].velocities = zeroVec;
	trajectory.points[0].accelerations = zeroVec;
	trajectory.points[0].time_from_start = ros::Duration(0.0);

	trajectory.points[1].positions = zeroVec;
    trajectory.points[1].positions[1] = paraMiddleHeadTilt;
    trajectory.points[1].positions[jointIndex] = desiredJointValue;
	trajectory.points[1].velocities = zeroVec;
	trajectory.points[1].accelerations = zeroVec;
	trajectory.points[1].time_from_start = ros::Duration(5.0);

	headGoal.trajectory = trajectory;
	headGoal.goal_time_tolerance = ros::Duration(0.0);

	controllerActionClient.sendGoal(headGoal);
}


void GoalPublisherWithHead::initializePose()
{
    controlHead(0.0, paraMiddleHeadTilt, 1);
}
