#include "SimLogReader.h"

SimLogReader::SimLogReader(std::string file, bool flipColors)
 : LogReader(file, flipColors),
   nh(),
   //rgbSub(nh, "/camera/rgb/image_raw", 100),
   //depthSub(nh, "/camera/depth/image_raw", 100),
   //odomSub(nh, "/odom", 10),
   //sync(rgbSub, depthSub, 30),
   sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry>(30), rgbSub, depthSub, odomSub)
   //originPose(nullptr),
{
	configureSubscriber();

	std::cout << "Creating simulated capture..." << std::endl;

	sync.registerCallback(boost::bind(&SimLogReader::syncCallback, this, _1, _2, _3));

	decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
	decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

	int waitCount = 0;
	if (depthSub.getSubscriber().getNumPublishers() == 0 || rgbSub.getSubscriber().getNumPublishers() == 0)
	{
		std::cout << "Waiting for the publisher...";
		do
		{
			usleep(33333);
			std::cout << ".";
			std::cout.flush();

			if (waitCount > 100)
				break;
			++waitCount;
		} while(depthSub.getSubscriber().getNumPublishers() == 0 || rgbSub.getSubscriber().getNumPublishers() == 0);

		if (waitCount > 100)
		{
			std::cerr << std::endl << "Error: no one publishes on this topic. Make sure depth & rgb topics are:" << std::endl
					  << depthTopic << std::endl
					  << rgbTopic << std::endl;

			isOk = false;
		}
		else
		{
			isOk = true;
		}
	}
	if (isOk)
	{
		std::cout << "sucess!" << std::endl;
		std::cout << "Waiting for first frame";
		std::cout.flush();

		do
		{
			usleep(33333);
			std::cout << ".";
			std::cout.flush();
			ros::spinOnce();
		} while(depthBuffer.size() == 0 || rgbBuffer.size() == 0);

		std::cout << " got it" << std::endl;
	}
	startTime = std::chrono::system_clock::now();
}

SimLogReader::~SimLogReader()
{
	delete [] decompressionBufferDepth;
	delete [] decompressionBufferImage;

	while (!rgbBuffer.empty())
	{
		delete [] rgbBuffer.front();
		delete [] depthBuffer.front();
		rgbBuffer.pop();
		depthBuffer.pop();
	}
	//if (originPose)
	//	delete originPose;
}

void SimLogReader::getNext()
{
	if (rgbBuffer.empty())
	{
		//std::cout << "Empty rgbd data!" << std::endl;
		ros::spinOnce();
		return;
	}

	memcpy(&decompressionBufferDepth[0], depthBuffer.front(), Resolution::getInstance().numPixels() * 2);
	memcpy(&decompressionBufferImage[0], rgbBuffer.front(), Resolution::getInstance().numPixels() * 3);
	
	timestamp = timeBuffer.front();
	currentPose = odomBuffer.front();

	delete [] depthBuffer.front();
	delete [] rgbBuffer.front();

	depthBuffer.pop();
	rgbBuffer.pop();
	timeBuffer.pop();
	odomBuffer.pop();

	rgb = (unsigned char *)&decompressionBufferImage[0];
	depth = (unsigned short *)&decompressionBufferDepth[0];

	imageReadBuffer = 0;
	depthReadBuffer = 0;

	imageSize = Resolution::getInstance().numPixels() * 3;
	depthSize = Resolution::getInstance().numPixels() * 2;

	if (flipColors)
	{
		for (int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
		{
			std::swap(rgb[i + 0], rgb[i + 2]);
		}
	}
	ros::spinOnce();
}

void SimLogReader::syncCallback(const sensor_msgs::Image::ConstPtr & rgbImg, const sensor_msgs::Image::ConstPtr & depthImg, const nav_msgs::Odometry::ConstPtr & odomPose)
{
	tf::StampedTransform transformSLAM;
	try
	{
		listener.waitForTransform("/base_link", rgbOpticalFrame, rgbImg->header.stamp, ros::Duration(1.0));
		listener.lookupTransform("/base_link", rgbOpticalFrame, rgbImg->header.stamp, transformSLAM);
	}
	catch (tf::TransformException &ex)
	{
		//ROS_ERROR("%s", ex.what());
		return;
	}

	//auto endTime = std::chrono::system_clock::now();
	//auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);
	//if (duration.count() < 1)
	//{
	//	++pubHz;	
	//}
	//else
	//{
	//	std::cout << pubHz << " frames per second" << std::endl;
	//	startTime = endTime;
	//	pubHz = 0;
	//}

	unsigned short * depthData = new unsigned short[Resolution::getInstance().numPixels()];
	if (depthImg->encoding == "32FC1")
	{
		// Convert depth data from 32FC1 to uint16_t
		const float * rawData = reinterpret_cast<const float *>(&depthImg->data[0]);
		unsigned short badPoint = 0;
		for (decltype(depthImg->height) i = 0; i < depthImg->height * depthImg->width; ++i)
		{
			float raw = rawData[i];
			depthData[i] = std::isnan(raw) ? badPoint : (unsigned short)(raw * 1000);
		}
	}
	else
	{
		memcpy(depthData, &depthImg->data[0], sizeof(unsigned short) * depthImg->height * depthImg->width);
	}

	unsigned char * rgbData = new unsigned char[Resolution::getInstance().numPixels() * 3];
	memcpy(rgbData, &rgbImg->data[0], sizeof(unsigned char) * rgbImg->height * rgbImg->width * 3);
	
	tf::Transform transformBase;

	Eigen::Matrix4f poseMat;
	poseMat.setIdentity();
	tf::Vector3 tfPoseVec(odomPose->pose.pose.position.x, odomPose->pose.pose.position.y, odomPose->pose.pose.position.z);
	tf::Quaternion tfQuat(odomPose->pose.pose.orientation.x, odomPose->pose.pose.orientation.y, odomPose->pose.pose.orientation.z, odomPose->pose.pose.orientation.w);
	transformBase.setOrigin(tfPoseVec);
	transformBase.setRotation(tfQuat);

	if (!isInitial)
	{
		originalTransform = transformBase * transformSLAM;
		originalTransform = originalTransform.inverse();
		isInitial = true;
	}
	//Eigen::Quaternionf quat((originTransform * transformSLAM.getRotation()).w(), (originTransform * transformSLAM.getRotation()).x(), 
	//		(originTransform * transformSLAM.getRotation()).y(), (originTransform * transformSLAM.getRotation()).z());
	//Eigen::Matrix3f rotMat = quat.toRotationMatrix();
	//Eigen::Vector3f transVec((originTransform * transformSLAM.getOrigin()).x(), (originTransform * transformSLAM.getOrigin()).y(), (originTransform * transformSLAM.getOrigin()).z());

	//Eigen::Quaternionf quat((transformSLAM * tfQuat).w(), (transformSLAM * tfQuat).x(), 
	//		(transformSLAM * tfQuat).y(), (transformSLAM * tfQuat).z());
	transformBase = transformBase * transformSLAM;
	Eigen::Quaternionf quat((originalTransform * transformBase).getRotation().w(), (originalTransform * transformBase).getRotation().x(), 
			(originalTransform * transformBase).getRotation().y(), (originalTransform * transformBase).getRotation().z());
	Eigen::Matrix3f rotMat = quat.toRotationMatrix();
	Eigen::Vector3f transVec((originalTransform * transformBase).getOrigin().x(), (originalTransform * transformBase).getOrigin().y(), (originalTransform * transformBase).getOrigin().z());

	poseMat.topRightCorner(3, 1) = transVec;
	poseMat.topLeftCorner(3, 3) = rotMat;

	//std::cout << poseMat << std::endl << std::endl;

	/*if (!originPose)
	{
		originPose = new Eigen::Matrix4f;
		*originPose = poseMat;
	}*/

	//poseMat.topRightCorner(3, 1) = poseMat.topRightCorner(3, 1) - originPose->topRightCorner(3, 1);
	//poseMat.topLeftCorner(3, 3) = poseMat.topLeftCorner(3, 3) * originPose->topLeftCorner(3, 3).transpose();
	//poseMat = poseMat * (*originPose).inverse();

	// convert to SLAM frame
	//Eigen::Matrix4f slamTransMat;
	//slamTransMat << 0, -1, 0, 0,
	//			 0, 0, -1, 0,
	//			 1, 0, 0, 0,
	//			 0, 0, 0, 1;

	//poseMat = slamTransMat * poseMat * slamTransMat.transpose();
	
	//std::cout << "Strategy1:" << std::endl;
	//std::cout << poseMat << std::endl;

	/*Eigen::Vector3f xOdom(1, 0, 0);
	Eigen::Vector3f yOdom(0, 1, 0);
	Eigen::Vector3f zOdom(0, 0, 1);

	Eigen::Vector3f xSlam = originPose->topLeftCorner(3, 3) * (-yOdom);
	Eigen::Vector3f ySlam = originPose->topLeftCorner(3, 3) * (-zOdom);
	Eigen::Vector3f zSlam = originPose->topLeftCorner(3, 3) * xOdom;

	Eigen::Matrix3f slamBaseMat;
	slamBaseMat.col(0) = xSlam.normalized();
	slamBaseMat.col(1) = ySlam.normalized();
	slamBaseMat.col(2) = xSlam.cross(ySlam);

	poseMat.topRightCorner(3, 1) = slamBaseMat.fullPivHouseholderQr().solve(transVec - originPose->topRightCorner(3, 1));

	Eigen::Matrix3f newSlamBaseMat;
	newSlamBaseMat.col(0) = slamBaseMat.fullPivHouseholderQr().solve((rotMat * (-yOdom)).normalized());
	newSlamBaseMat.col(1) = slamBaseMat.fullPivHouseholderQr().solve((rotMat * (-zOdom)).normalized());
	newSlamBaseMat.col(2) = slamBaseMat.fullPivHouseholderQr().solve((rotMat * xOdom).normalized());

	poseMat.topLeftCorner(3, 3) = newSlamBaseMat;*/
	//std::cout << newSlamBaseMat * newSlamBaseMat.transpose() << std::endl;
	//std::cout << "Strategy2:" << std::endl;
	//std::cout << poseMat << std::endl;
	
	if (rgbBuffer.size() < maxBufferNum)
	{
		depthBuffer.push(depthData);
		rgbBuffer.push(rgbData);
		timeBuffer.push((int64_t)rgbImg->header.stamp.toNSec());
		odomBuffer.push(poseMat);
	}
	else
	{
		delete [] depthBuffer.front();
		delete [] rgbBuffer.front();

		depthBuffer.pop();
		rgbBuffer.pop();
		timeBuffer.pop();
		odomBuffer.pop();

		depthBuffer.push(depthData);
		rgbBuffer.push(rgbData);
		timeBuffer.push((int64_t)rgbImg->header.stamp.toNSec());
		odomBuffer.push(poseMat);
	}
}

const std::string SimLogReader::getFile()
{
	return Parse::get().baseDir().append("sim");
}

int SimLogReader::getNumFrames()
{
	return std::numeric_limits<int>::max();
}

bool SimLogReader::hasMore()
{
	return true;
}

void SimLogReader::setAuto(bool value)
{

}

bool SimLogReader::ok()
{
	return isOk;
}

void SimLogReader::configureSubscriber()
{
	std::ifstream simConfigFile(file.c_str());

	std::getline(simConfigFile, rgbTopic);
   	rgbSub.subscribe(nh, rgbTopic, 30);

	std::getline(simConfigFile, depthTopic);
   	depthSub.subscribe(nh, depthTopic, 30);

	std::getline(simConfigFile, odomTopic);
   	odomSub.subscribe(nh, odomTopic, 30);

	std::getline(simConfigFile, rgbOpticalFrame);
}
