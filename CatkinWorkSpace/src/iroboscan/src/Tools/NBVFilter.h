#ifndef NBV_FILTER_H_
#define NBV_FILTER_H_

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

class NBVFilter
{
	public:
		NBVFilter(const std::string & mapTopic);
		~NBVFilter() = default;

		bool isValid(const tf::Vector3 & goalPosition, const tf::Vector3 & lookAtPosition, const tf::Transform & transformToMapFrame);

	private:
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & mapMsg);

		ros::NodeHandle nh;
		ros::Subscriber mapSubscriber;

		nav_msgs::OccupancyGrid mapData;
		
};

#endif /* NBV_FILTER_H_ */
