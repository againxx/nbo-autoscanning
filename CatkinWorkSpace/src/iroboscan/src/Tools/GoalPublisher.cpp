#include "GoalPublisher.h"

const int GoalPublisher::PROCESSING = 0;
const int GoalPublisher::REPLANING = 1;
const int GoalPublisher::SUCCEEDED = 2;

GoalPublisher::GoalPublisher(const std::string & moveBaseTopic,
							 const std::string & basePoseTopic,
							 const std::string & controllerTopic)
 : nh(),
   moveBaseActionClient(moveBaseTopic, true),
   basePoseSubscriber(nh.subscribe(basePoseTopic, 30, &GoalPublisher::basePoseCallback, this)),
   controllerActionClient(controllerTopic, true)
{
	moveBaseGoalPose.header.frame_id = "/map";

	ROS_INFO("Waiting for move base action server to start.");
	moveBaseActionClient.waitForServer();
	ROS_INFO("Move base action server started."); 

	ROS_INFO("Waiting for controller action server to start.");
	controllerActionClient.waitForServer();
	ROS_INFO("Controller action server started."); 
}

void GoalPublisher::publishMoveBaseGoal()
{
	computeMoveBaseGoal();
	sendMoveBaseGoal();
}

void GoalPublisher::sendMoveBaseGoal()
{
	move_base_msgs::MoveBaseGoal moveBaseGoal;
	moveBaseGoal.target_pose = moveBaseGoalPose;

	moveBaseActionClient.sendGoal(moveBaseGoal);
}

void GoalPublisher::setGoal(const Eigen::Vector3f & lookAtPos, const Eigen::Vector3f & goalPos, const tf::Transform & transform)
{
	lookAtPosition = lookAtPos;
	goalPosition = goalPos;
	transformToMapFrame = transform;
	state = READY;
}

void GoalPublisher::basePoseCallback(const nav_msgs::Odometry::ConstPtr & odomMsg)
{
	basePose.setIdentity();

	Eigen::Quaternionf quat(odomMsg->pose.pose.orientation.w, odomMsg->pose.pose.orientation.x, 
							odomMsg->pose.pose.orientation.y, odomMsg->pose.pose.orientation.z);
	Eigen::Matrix3f rotMat = quat.toRotationMatrix();
	Eigen::Vector3f transVec(odomMsg->pose.pose.position.x, odomMsg->pose.pose.position.y, odomMsg->pose.pose.position.z);

	basePose.topRightCorner(3, 1) = transVec;
	basePose.topLeftCorner(3, 3) = rotMat;
}
