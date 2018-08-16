#include "ROSLiveLogReader.h"

ROSLiveLogReader::ROSLiveLogReader(std::string file, bool flipColors)
 : LogReader(file, flipColors),
   nh(),
   //sync(rgbSub, depthSub, 30)
   sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(1000), rgbSub, depthSub)
{
	configureSubscriber();

	std::cout << "Creating ROS live capture..." << std::endl;

	sync.registerCallback(boost::bind(&ROSLiveLogReader::syncCallback, this, _1, _2));

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

ROSLiveLogReader::~ROSLiveLogReader()
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
}

void ROSLiveLogReader::getNext()
{
	if (rgbBuffer.empty())
	{
		//std::cout << "Error: empty rgb data!" << std::endl;
		ros::spinOnce();
		return;
	}

	memcpy(&decompressionBufferDepth[0], depthBuffer.front(), Resolution::getInstance().numPixels() * 2);
	memcpy(&decompressionBufferImage[0], rgbBuffer.front(), Resolution::getInstance().numPixels() * 3);
	
	timestamp = timeBuffer.front();

	delete [] depthBuffer.front();
	delete [] rgbBuffer.front();

	depthBuffer.pop();
	rgbBuffer.pop();
	timeBuffer.pop();

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

void ROSLiveLogReader::syncCallback(const sensor_msgs::Image::ConstPtr & rgbImg, const sensor_msgs::Image::ConstPtr & depthImg)
{
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
	memcpy(depthData, &depthImg->data[0], sizeof(unsigned short) * depthImg->height * depthImg->width);

	unsigned char * rgbData = new unsigned char[Resolution::getInstance().numPixels() * 3];
	memcpy(rgbData, &rgbImg->data[0], sizeof(unsigned char) * rgbImg->height * rgbImg->width * 3);

	if (rgbBuffer.size() < maxBufferNum)
	{
		depthBuffer.push(depthData);
		rgbBuffer.push(rgbData);
		timeBuffer.push((int64_t)rgbImg->header.stamp.toNSec());
	}
	else
	{
		delete [] depthBuffer.front();
		delete [] rgbBuffer.front();

		depthBuffer.pop();
		rgbBuffer.pop();
		timeBuffer.pop();

		depthBuffer.push(depthData);
		rgbBuffer.push(rgbData);
		timeBuffer.push((int64_t)rgbImg->header.stamp.toNSec());
	}
}

const std::string ROSLiveLogReader::getFile()
{
	return Parse::get().baseDir().append("roslive");
}

int ROSLiveLogReader::getNumFrames()
{
	return std::numeric_limits<int>::max();
}

bool ROSLiveLogReader::hasMore()
{
	return true;
}

void ROSLiveLogReader::setAuto(bool value)
{

}

bool ROSLiveLogReader::ok()
{
	return isOk;
}

void ROSLiveLogReader::configureSubscriber()
{
	std::ifstream liveConfigFile(file.c_str());

	std::getline(liveConfigFile, rgbTopic);
   	rgbSub.subscribe(nh, rgbTopic, 1000);

	std::getline(liveConfigFile, depthTopic);
   	depthSub.subscribe(nh, depthTopic, 1000);
}
