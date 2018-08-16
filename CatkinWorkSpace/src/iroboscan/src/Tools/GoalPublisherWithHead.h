#ifndef GOAL_PUBLISHER_WITH_HEAD_H_
#define GOAL_PUBLISHER_WITH_HEAD_H_

#include "GoalPublisher.h"

class GoalPublisherWithHead : public GoalPublisher
{
	public:
		GoalPublisherWithHead(const std::string & moveBaseTopic,
					  		  const std::string & basePoseTopic,
					  		  const std::string & controllerTopic);

		~GoalPublisherWithHead() = default;

		int mainLoop();

		void initializePose();

	private:
		void computeMoveBaseGoal();

		void controlHead(double currentJointValue, double desiredJointValue, int jointIndex);

		std::chrono::system_clock::time_point beginPausingTime;

		double paraUpHeadTilt = -0.6;
        double paraMiddleHeadTilt = 0.5;
		double paraDownHeadTilt = 1.0;
		double paraLeftHeadPan = 0.6;
        double paraMiddleHeadPan = 0.0;
		double paraRightHeadPan = -0.6;

		bool arrivedUp = false;
		bool arrivedDown = false;
		bool arrivedRight = false;
		bool arrivedLeft = false;

		static const int LOOKING_UP_DOWN = 4;
		static const int LOOKING_LEFT_RIGHT = 5;
};

#endif /* GOAL_PUBLISHER_WITH_HEAD_H_ */
