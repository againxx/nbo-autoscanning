#ifndef ROS_LIVE_LOG_READER_H_
#define ROS_LIVE_LOG_READER_H_

#include <iostream>
#include <queue>
#include <chrono>
#include <fstream>
#include <string>
#include <Utils/Parse.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include "LogReader.h"

class ROSLiveLogReader : public LogReader
{
	public:
		ROSLiveLogReader(std::string file, bool flipColors);

		virtual ~ROSLiveLogReader();

		void getNext();
		
		int getNumFrames();

		bool hasMore();

		bool rewound()
		{
			return false;
		}

		void rewind()
		{
		
		}

		void getBack()
		{
		
		}

		void fastForward(int frame)
		{
		
		}

		const std::string getFile();

		void setAuto(bool value);

		bool ok();
		
	private:
		void syncCallback(const sensor_msgs::Image::ConstPtr & rgbImg, const sensor_msgs::Image::ConstPtr & depthImg);

		void configureSubscriber();

	private:
		ros::NodeHandle nh;

		message_filters::Subscriber<sensor_msgs::Image> rgbSub;
		message_filters::Subscriber<sensor_msgs::Image> depthSub;
		//message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync;
		message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>> sync;

		std::queue<unsigned short *> depthBuffer;
		std::queue<unsigned char *> rgbBuffer;
		std::queue<int64_t>	timeBuffer;

		decltype(rgbBuffer.size()) maxBufferNum = 30;
		
		bool isOk = false;

		std::chrono::system_clock::time_point startTime;
		int pubHz = 0;

		std::string rgbTopic;
		std::string depthTopic;
};

#endif /* ROS_LIVE_LOG_READER_H_ */
