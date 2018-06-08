#include "NBO.h"

NBO::NBO(std::shared_ptr<PartialMatchingFacada> pmFacada)
 : partialMatchingFacada(pmFacada)
{   

}

void NBO::computeNBO(const Eigen::Matrix4f & currPose, const std::vector<SegmentedPatch> & objectHypo)
{
	nboFlag = false;
	recognizedObjects.clear();

	double currBestPoint = 0;
	Eigen::Vector3f camPos = currPose.topRightCorner(3,1);
	Eigen::Matrix3f camRot = currPose.topLeftCorner(3,3);
	Eigen::Quaternionf camQuat(camRot);
	Eigen::Vector3f forwardVector(0, 0, 1);
	Eigen::Vector3f camForward = (camQuat * forwardVector).normalized();

	std::cout << "Object Hypotheses size: " << objectHypo.size() << std::endl;
	if (objectHypo.empty())
		return;

	std::vector<double> objectnessForObjectHypo(objectHypo.size(), 0);
	std::vector<int> pointNumOfObjectHypo(objectHypo.size(), 0);
	std::vector<MatchingResultParser> matchingResults(objectHypo.size());
	for (size_t i = 0; i < objectHypo.size(); ++i)
	{
		std::cout << std::endl;
		std::cout << "Hypothesis: ";
		for (auto index : objectHypo[i].getLabelIndexes())
			std::cout << index << " ";
		std::cout << std::endl;

		MatchingResultParser parser = partialMatchingFacada->getMatchingResult(objectHypo[i], true);
		matchingResults[i] = parser;
		if (parser.matchNum == 0)
			continue;

		double objectnessValue = 0;
		bool recogNewObject = false;
		for (int j = 0; j < parser.matchNum; ++j)
		{
			std::cout << parser.modelNames[j] << "  error:" << parser.errors[j];
			if (parser.errors[j] < paraMaxErrorConsiderRecognized * objectHypo[i].getBoundingBox().getDiameter() && !recogNewObject)
			{
				recognizedObjects.push_back(objectHypo[i]);
				recogNewObject = true;
				break;
			}

			double confidence = gaussFunction(parser.errors[j] / objectHypo[i].getBoundingBox().getDiameter());
			std::cout << "  confidence:" << confidence << std::endl;
			objectnessValue = std::max(objectnessValue, confidence);
		}

		if (recogNewObject)
		{
			std::cout << "Recognized new object!" << std::endl;
			continue;
		}
		else
		{
			objectnessForObjectHypo[i] = objectnessValue;
			pointNumOfObjectHypo[i] = objectHypo[i].getPointNum();
		}
	}

	int maxPointNum = *(std::max_element(pointNumOfObjectHypo.cbegin(), pointNumOfObjectHypo.cend()));
	for (size_t i = 0; i < objectHypo.size(); ++i)
	{
		if (objectnessForObjectHypo[i] == 0)
			continue;

		Eigen::Vector3f patchCenter = objectHypo[i].getCenterPosition() - camPos;
		double orientValue = ((camForward.dot(patchCenter.normalized()) + 1.0) * 0.5);
		double positionValue = exp(-pow(patchCenter.norm(), 2) / 2);
		
		if (paraObjectnessWeight * objectnessForObjectHypo[i]
		  + paraOrientWeight * orientValue
		  + paraPositionWeight * positionValue 
		  + paraSizeWeight * pointNumOfObjectHypo[i] / static_cast<double>(maxPointNum) > currBestPoint)
		{
			currBestPoint = paraObjectnessWeight * objectnessForObjectHypo[i]
						  + paraOrientWeight * orientValue
						  + paraPositionWeight * positionValue
						  + paraSizeWeight * pointNumOfObjectHypo[i] / static_cast<double>(maxPointNum);

			nbo = objectHypo[i];
			nboMatchingResult = matchingResults[i];
			nboFlag = true;
		}
	}
}

inline double NBO::gaussFunction(double x)
{
	return exp(-pow(x - paraGaussFunctionCenter, 2) / (2 * pow(paraGaussFunctionWidth, 2)));
}
