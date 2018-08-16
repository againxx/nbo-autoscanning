#ifndef SCANNING_CONTROLLER_H_
#define SCANNING_CONTROLLER_H_

#include <vector>
#include <thread>
#include <ElasticFusion.h>
#include <ScanningScheme.h>
#include "GoalPublisherWithArm.h"
#include "GoalPublisherWithHead.h"
#include "NBOFlicker.h"
#include "NBVFilter.h"
#include "YoloWrapper.h"
#include "SimLogReader.h"
#include "GUI.h"

class ScanningController
{
	public:
		ScanningController(ElasticFusion * fusion, SimLogReader * logReader, GUI * ui);
		ScanningController(ElasticFusion * fusion, GUI * ui);
		~ScanningController();

		void loop();

		void stepScan();
		void stopScan();
		void autoScan();

		void renderNBOMatchingModels(pangolin::OpenGlMatrix mvp);
		void renderNBOPosition();
		void renderNBVPose();
		void renderNBVTrajectory();

		void popCurrentNBV();
		void initializePose();
		void saveRecognizedObjects();

		// IS functions
		bool blockFusion() { return (state == FLICKERING || state == COMPUTING) ? true : false; }

	private:
		void computeNBOAndNBV();
		void computeNBVPose();
		void filterCollidedNBVPoses();
		void clearGoal();

		int mode = STOP;
		int state = READY;

		bool changeView = true;

		ScanningScheme * scanningScheme = nullptr;
		GoalPublisher * goalPublisher = nullptr;
		NBOFlicker * nboFlicker = nullptr;
		NBVFilter * nbvFilter = nullptr;
		YoloWrapper * yoloWrapper = nullptr;
		ElasticFusion * eFusion = nullptr;
		SimLogReader * simLogReader = nullptr;
		GUI * gui = nullptr;

		std::chrono::system_clock::time_point beginPausingTime;

		std::queue<std::array<float, 6>> nbvGoalsQueue;
		std::vector<Eigen::Matrix4f> nbvTrajectory;

		Eigen::Vector3f nboPosition;
		Eigen::Matrix4f nbvPose;

		static const int STOP = 0;
		static const int STEP = 1;
		static const int AUTO = 2;

		static const int READY = 0;
		static const int COMPUTING = 1;
		static const int FLICKERING = 2;
		static const int NO_GOAL = 3;
		static const int MOVING = 4;
		static const int PAUSING = 5;
		static const int COMPLETE = 6;
};

#endif /* SCANNING_CONTROLLER_H_ */
