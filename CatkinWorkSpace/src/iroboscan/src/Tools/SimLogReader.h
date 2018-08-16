#ifndef SIMLOGREADER_H_
#define SIMLOGREADER_H_

#include <iostream>
#include <queue>
#include <Utils/Parse.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <chrono>
#include <fstream>
#include <string>
#include "LogReader.h"

class SimLogReader : public LogReader
{
	public:
		SimLogReader(std::string file, bool flipColors);

		virtual ~SimLogReader();

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
		
		// GET functions
		const Eigen::Matrix4f & getCurrentPose() const { return currentPose; }
		const tf::Transform & getOriginalTransform() const { return originalTransform; }

	private:
		void syncCallback(const sensor_msgs::Image::ConstPtr & rgbImg, const sensor_msgs::Image::ConstPtr & depthImg, const nav_msgs::Odometry::ConstPtr & odomPose);

		void configureSubscriber();

	private:
		ros::NodeHandle nh;
		tf::TransformListener listener;
		message_filters::Subscriber<sensor_msgs::Image> rgbSub;
		message_filters::Subscriber<sensor_msgs::Image> depthSub;
		message_filters::Subscriber<nav_msgs::Odometry> odomSub;
		//message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync;
		message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry>> sync;
		//message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, gazebo_msgs::LinkStates> sync;

		//Eigen::Matrix4f * originPose;
		bool isInitial = false;

		std::queue<unsigned short *> depthBuffer;
		std::queue<unsigned char *> rgbBuffer;
		std::queue<int64_t>	timeBuffer;
		std::queue<Eigen::Matrix4f> odomBuffer;
		std::chrono::system_clock::time_point startTime;
		int pubHz = 0;
		decltype(rgbBuffer.size()) maxBufferNum = 30;
		bool isOk = false;

		std::string rgbTopic;
		std::string depthTopic;
		std::string odomTopic;
		std::string rgbOpticalFrame;

		Eigen::Matrix4f currentPose;
		tf::Transform originalTransform;
};

#endif /* SIMLOGREADER_H_ */
