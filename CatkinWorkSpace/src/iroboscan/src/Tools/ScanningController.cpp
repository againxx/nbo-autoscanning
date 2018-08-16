#include "ScanningController.h"
#include "GUI.h"

ScanningController::ScanningController(ElasticFusion * fusion, SimLogReader * logReader, GUI * ui)
 : eFusion(fusion),
   simLogReader(logReader),
   gui(ui)
{
	std::string homePath = getenv("HOME");
	scanningScheme = new ScanningScheme(homePath + "/ProjectData/SunCG/house_01/PatchData/keypoint", homePath + "/ProjectData/SunCG/house_01/PatchData/Patch");
	//goalPublisher = new GoalPublisherWithArm("/move_base", "/base_pose_ground_truth", "/arm_with_torso_controller/follow_joint_trajectory", "arm_with_torso");
	goalPublisher = new GoalPublisherWithHead("/move_base", "/base_pose_ground_truth", "/head_controller/follow_joint_trajectory");
	nboFlicker = new NBOFlicker();
	nbvFilter = new NBVFilter("/map_keepout");
	//yoloWrapper = new YoloWrapper();
}

ScanningController::ScanningController(ElasticFusion * fusion, GUI * ui)
 : eFusion(fusion),
   gui(ui)
{
	std::string homePath = getenv("HOME");
	//scanningScheme = new ScanningScheme(homePath + "/ProjectData/Room2_ModelKeypointData", homePath + "/ProjectData/Room2_Patch");
	//goalPublisher = new GoalPublisher("/move_base", "arm_with_torso", "/base_pose_ground_truth", "/arm_with_torso_controller/follow_joint_trajectory");
	nboFlicker = new NBOFlicker();
	nbvFilter = new NBVFilter("/map_keepout");
	//yoloWrapper = new YoloWrapper();
}

ScanningController::~ScanningController()
{
	if (scanningScheme)
		delete scanningScheme;
	if (goalPublisher)
		delete goalPublisher;
	if (nboFlicker)
		delete nboFlicker;
	if (nbvFilter)
		delete nbvFilter;
	if (yoloWrapper)
		delete yoloWrapper;
}

void ScanningController::stepScan()
{
	if (scanningScheme)
	{
		mode = STEP;
		state = READY;
	}
}

void ScanningController::stopScan()
{
	if (mode == AUTO)
		mode = STOP;
}

void ScanningController::autoScan()
{
	if (scanningScheme && mode != AUTO && simLogReader)
	{
		mode = AUTO;
		if (state == NO_GOAL)
			state = READY;
	}
}

void ScanningController::loop()
{
	if (mode == STOP)
		return;

	switch (state)
	{
		case READY:
			computeNBOAndNBV();	
			state = COMPUTING;
			break;
		case COMPUTING:
			if (!scanningScheme->isProcessing())
			{
				if (scanningScheme->hasRecognizedUnmergedPatches())
				{
					eFusion->mergeRecognizedPatches(scanningScheme->getRecognizedUnmergedPatchLabels());
					scanningScheme->clearRecognizedUnmergedPatches();
				}
				eFusion->setGroundPlainLabels(scanningScheme->getGroundPlainLabels());

				if (scanningScheme->hasGoal())
				{
					nbvGoalsQueue = scanningScheme->getNBVGoals();
					if (simLogReader)
						filterCollidedNBVPoses();

					if (state == NO_GOAL)
						break;

					computeNBVPose();

					if (changeView)
					{
						gui->followPose->Ref().Set(false);
						gui->godView->Ref().Set(true);
					}

					if (gui->drawNBO->Get())
					{
						nboFlicker->flicker(eFusion->getGlobalModel().lastCount(), eFusion->getGlobalModel().downloadMap(), scanningScheme->getNBOPatch().getLabelIndexes());
						eFusion->getGlobalModel().uploadMap(nboFlicker->getMapData());
						state = FLICKERING;
					}
					else
					{
						if (simLogReader)
						{
							goalPublisher->setGoal(nboPosition,
												   nbvPose.topRightCorner(3, 1),
												   simLogReader->getOriginalTransform().inverse());
							state = MOVING;
						}
						else
							state = COMPLETE;
					}
				}
				else
					state = NO_GOAL;
			}
			break;
		case FLICKERING:
			if (nboFlicker->finishFlickering())
			{
				eFusion->getGlobalModel().uploadMap(nboFlicker->getMapData());
				nboFlicker->clearMapData();

				if (simLogReader && mode == AUTO)
				{
					goalPublisher->setGoal(nboPosition,
										   nbvPose.topRightCorner(3, 1),
										   simLogReader->getOriginalTransform().inverse());
					state = MOVING;
				}
				else
					state = COMPLETE;

			}
			else if (nboFlicker->needSwitch())
			{
				eFusion->getGlobalModel().uploadMap(nboFlicker->getMapData());
			}
			break;
		case MOVING:
			{
				auto movingState = goalPublisher->mainLoop();
				if (movingState == GoalPublisher::REPLANING)
				{
					nbvGoalsQueue.pop();
					if (nbvGoalsQueue.empty())
					{
						std::cerr << "Error: nbv queue is empty!" << std::endl;
						state = NO_GOAL;
						break;
					}

					computeNBVPose();

					goalPublisher->setGoal(nboPosition,
								   		   nbvPose.topRightCorner(3, 1),
								   		   simLogReader->getOriginalTransform().inverse());
				}
				else if (movingState == GoalPublisher::SUCCEEDED)
				{
					if (changeView)
					{
						gui->followPose->Ref().Set(true);
						gui->godView->Ref().Set(false);
					}

					beginPausingTime = std::chrono::system_clock::now();
					nbvTrajectory.push_back(nbvPose);
					state = PAUSING;
				}
			}
			break;
		case PAUSING:
			if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - beginPausingTime).count() > 10)
			{
				scanningScheme->clearGoal();
				clearGoal();

				state = COMPLETE;
			}
			break;
		case COMPLETE:
			if (mode == AUTO)
				state = READY;
			break;
		case NO_GOAL:
		default:
			break;
	}
}

void ScanningController::computeNBOAndNBV()
{
	scanningScheme->clearGoal();

	std::thread processThread;
	if (yoloWrapper && gui->useYolo->Get())
	{
		yoloWrapper->process(simLogReader->rgb, eFusion->getProjectedLabels(), false);

		processThread = std::thread(static_cast<void(ScanningScheme::*)(int, Eigen::Vector4f *, float, const Eigen::Matrix4f &, std::vector<unsigned short>, double)>(&ScanningScheme::process),
									scanningScheme,
									eFusion->getGlobalModel().lastCount(),
									eFusion->getGlobalModel().downloadMap(),
									gui->confidenceThreshold->Get(),
									std::cref(eFusion->getCurrPose()),
									yoloWrapper->getRecognizedPatchesByYolo(),
									yoloWrapper->getProbability());
	}
	else
	{
		processThread = std::thread(static_cast<void(ScanningScheme::*)(int, Eigen::Vector4f *, float, const Eigen::Matrix4f &, std::vector<unsigned short>, double)>(&ScanningScheme::process),
									scanningScheme,
									eFusion->getGlobalModel().lastCount(),
									eFusion->getGlobalModel().downloadMap(),
									gui->confidenceThreshold->Get(),
									std::cref(eFusion->getCurrPose()),
									std::vector<unsigned short>(),
									0.0);
	}

	if (processThread.joinable())
		processThread.detach();
}

void ScanningController::computeNBVPose()
{
	Eigen::Vector3f nbvPosition;
	for (int i = 0; i < 3; ++i)
	{
		nbvPosition[i] = nbvGoalsQueue.front()[i];
		nboPosition[i] = nbvGoalsQueue.front()[3 + i];
	}

	Eigen::Vector3f upVector(0, -1, 0);
	Eigen::Matrix3f nbvRotMat;

	Eigen::Vector3f nbvZ = (nboPosition - nbvPosition).normalized();
	Eigen::Vector3f nbvX = nbvZ.cross(upVector).normalized();
	Eigen::Vector3f nbvY = nbvZ.cross(nbvX).normalized();

	nbvRotMat.col(0) = nbvX;
	nbvRotMat.col(1) = nbvY;
	nbvRotMat.col(2) = nbvZ;

	nbvPose.setIdentity();
	nbvPose.topLeftCorner(3, 3) = nbvRotMat;
	nbvPose.topRightCorner(3, 1) = nbvPosition;
}

void ScanningController::filterCollidedNBVPoses()
{
	std::queue<std::array<float, 6>> tempNBVGoalsQueue;

	while (!nbvGoalsQueue.empty())
	{
		tf::Vector3 nbvPosition(nbvGoalsQueue.front()[0], nbvGoalsQueue.front()[1], nbvGoalsQueue.front()[2]);
		tf::Vector3 nboPosition(nbvGoalsQueue.front()[3], nbvGoalsQueue.front()[4], nbvGoalsQueue.front()[5]);
		if (nbvFilter->isValid(nbvPosition, nboPosition, simLogReader->getOriginalTransform().inverse()))
			tempNBVGoalsQueue.push(nbvGoalsQueue.front());

		nbvGoalsQueue.pop();
	}

	nbvGoalsQueue = tempNBVGoalsQueue;
	if (nbvGoalsQueue.empty())
	{
		std::cerr << "Error: nbv queue is empty!" << std::endl;
		state = NO_GOAL;
	}
}

void ScanningController::clearGoal()
{
	while (!nbvGoalsQueue.empty())
		nbvGoalsQueue.pop();

	nbvPose.setZero();
	nboPosition.setZero();
}

void ScanningController::renderNBVPose()
{
	if (!nbvGoalsQueue.empty())
		gui->drawFrustum(nbvPose);
}

void ScanningController::renderNBOPosition()
{
	if (!nbvGoalsQueue.empty())
		gui->drawCross(nboPosition);
}

void ScanningController::renderNBOMatchingModels(pangolin::OpenGlMatrix mvp)
{
	if (scanningScheme && scanningScheme->hasGoal())
		scanningScheme->renderNBOMatchingModels(mvp);
}

void ScanningController::renderNBVTrajectory()
{
	for (size_t i = 0; i < nbvTrajectory.size(); ++i)
	{
		glColor3f(0, i * 1.0 / (nbvTrajectory.size() - 1), 1);
		gui->drawFrustum(nbvTrajectory[i]);
	}
}

void ScanningController::popCurrentNBV()
{
	nbvGoalsQueue.pop();

	if (nbvGoalsQueue.empty())
	{
		std::cerr << "Error: nbv queue is empty!" << std::endl;
		state = NO_GOAL;
	}
	else
		computeNBVPose();
}

void ScanningController::initializePose()
{
	if (goalPublisher)
		goalPublisher->initializePose();
}

void ScanningController::saveRecognizedObjects()
{
	if (scanningScheme)
	{
		std::string homePath = getenv("HOME");
		scanningScheme->saveRecognizedObjects(homePath + "/ProjectData/RecognizedObjects/");
	}
}
