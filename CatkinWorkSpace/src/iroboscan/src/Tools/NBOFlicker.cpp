#include "NBOFlicker.h"

NBOFlicker::~NBOFlicker()
{
	if (originalMapData)
		delete [] originalMapData;
	if (modifiedMapData)
		delete [] modifiedMapData;
}

void NBOFlicker::flicker(int pointNum, Eigen::Vector4f * mapData, const std::vector<unsigned short> & nboPatchIndexes)
{
	if (originalMapData)
		delete [] originalMapData;
	if (modifiedMapData)
		delete [] modifiedMapData;

	originalMapData = mapData;
	modifiedMapData = new Eigen::Vector4f[pointNum * 3];
	memcpy(modifiedMapData, originalMapData, sizeof(Eigen::Vector4f) * 3 * pointNum);

	for (int i = 0; i < pointNum; ++i)
	{
		Eigen::Vector4f & col = modifiedMapData[3 * i + 1];
		int label = static_cast<int>(col[1]) >> 8 & 0xFFFF;
		if (label != 0)
		{
			if (std::find(nboPatchIndexes.cbegin(), nboPatchIndexes.cend(), label) != nboPatchIndexes.cend())
			{
				col[1] = static_cast<float>(65535 << 8);
			}
		}
	}

	beginFlickering = true;
	showOriginal = false;
	beginFlickeringTime = std::chrono::system_clock::now();
}

void NBOFlicker::clearMapData()
{
	if (originalMapData)
	{
		delete [] originalMapData;
		originalMapData = NULL;
	}
	if (modifiedMapData)
	{
		delete [] modifiedMapData;
		modifiedMapData = NULL;
	}
}

const Eigen::Vector4f * NBOFlicker::getMapData()
{
	lastSwitchTime = std::chrono::system_clock::now();

	if (showOriginal)
		return originalMapData;
	else
		return modifiedMapData;
}

bool NBOFlicker::finishFlickering()
{
	if (beginFlickering && 
		std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - beginFlickeringTime).count() > paraFlickeringDuration)
	{
		beginFlickering = false;
		showOriginal = true;
		return true;
	}
	else
	{
		return false;
	}
}

bool NBOFlicker::needSwitch()
{
	if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastSwitchTime).count() > paraSwitchingDuration)
	{
		showOriginal = !showOriginal;
		return true;
	}
	else
		return false;
}
