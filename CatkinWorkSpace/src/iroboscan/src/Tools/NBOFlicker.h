#ifndef NBO_FLICKER_H_
#define NBO_FLICKER_H_

#include <chrono>
#include <vector>
#include <Eigen/Core>

class NBOFlicker
{
	public:
		NBOFlicker() = default;
		~NBOFlicker();

		void flicker(int pointNum, Eigen::Vector4f * mapData, const std::vector<unsigned short> & nboPatchIndexes);

		void clearMapData();

		// GET functions
		const Eigen::Vector4f * getMapData();

		// IS functions
		bool needSwitch();
		bool finishFlickering();

	private:
		Eigen::Vector4f * originalMapData = nullptr;
		Eigen::Vector4f * modifiedMapData = nullptr;

		bool beginFlickering = false;
		bool showOriginal = false;

		std::chrono::system_clock::time_point beginFlickeringTime;
		std::chrono::system_clock::time_point lastSwitchTime;

		int paraSwitchingDuration = 500;
		int paraFlickeringDuration = 5000;
		
};

#endif /* NBO_FLICKER_H_ */
