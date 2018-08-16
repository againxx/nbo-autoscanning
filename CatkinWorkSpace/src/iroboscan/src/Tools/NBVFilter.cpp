#include "NBVFilter.h"

NBVFilter::NBVFilter(const std::string & mapTopic)
 : mapSubscriber(nh.subscribe(mapTopic, 30, &NBVFilter::mapCallback, this))
{

}

bool NBVFilter::isValid(const tf::Vector3 & goalPosition, const tf::Vector3 & lookAtPosition, const tf::Transform & transformToMapFrame)
{
	if (mapSubscriber.getNumPublishers() == 0)
		return true;

	tf::Vector3 mapGoalPos = transformToMapFrame * goalPosition;
	tf::Vector3 mapLookAtPos = transformToMapFrame * lookAtPosition;
	mapGoalPos.setZ(0.0);
	mapLookAtPos.setZ(0.0);

	//mapGoalPos -= 0.65 * (mapLookAtPos - mapGoalPos).normalized();

	int mapGoalX = (mapGoalPos.x() - mapData.info.origin.position.x) / mapData.info.resolution;
	int mapGoalY = (mapGoalPos.y() - mapData.info.origin.position.y) / mapData.info.resolution;

	std::cout << "Map goal: " << mapGoalX << " " << mapGoalY << " ";

	for (int i = mapGoalX - 6; i < mapGoalX + 6; ++i)
	{
		if (i < 0 || i >= static_cast<int>(mapData.info.height))
		{
			std::cout << "False" << std::endl;
			return false;
		}

		for (int j = mapGoalY - 6; j < mapGoalY + 6; ++j)
		{
			if (j < 0 || j >= static_cast<int>(mapData.info.width))
			{
				std::cout << "False" << std::endl;
				return false;
			}

			if (mapData.data[j * mapData.info.width + i] != 0)
			{
				std::cout << "False" << std::endl;
				return false;
			}
		}
	}

	std::cout << "True" << std::endl;
	return true;
}

void NBVFilter::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & mapMsg)
{
	mapData.header = mapMsg->header;
	mapData.info = mapMsg->info;
	mapData.data = mapMsg->data;
}
