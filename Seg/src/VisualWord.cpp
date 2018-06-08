#include "VisualWord.h"

const int VisualWord::HARD_ASSIGNMENT = 0;
const int VisualWord::SOFT_ASSIGNMENT = 1;

VisualWord::VisualWord(size_t num)
 : clusterNum(num)
{

}

VisualWord::~VisualWord()
{

}

void VisualWord::kmeansCluster(std::vector<KeypointRepresentation> & keyptReprs)
{
	if (keyptReprs.empty())
	{
		std::cerr << "Error: there is no model keypoint representation!" << std::endl;
		return;
	}
	clusterCenters.resize(clusterNum);
	
	int descriptorSize = KeypointRepresentation::descriptorSize;
	int totalKeypointNum = 0;
	for (const auto & keyptRepr : keyptReprs)
		totalKeypointNum += keyptRepr.getNum();

	cv::Mat labels, centers, descriptorData(totalKeypointNum, descriptorSize, CV_32F);

	size_t pointerShift = 0;
	for (size_t i = 0; i < keyptReprs.size(); ++i)
	{
		memcpy(descriptorData.data + pointerShift, keyptReprs[i].getDescriptors(), sizeof(float) * keyptReprs[i].getNum() * descriptorSize);
		pointerShift += keyptReprs[i].getNum() * descriptorSize * sizeof(float);
	}

	double compactness = cv::kmeans(descriptorData, clusterNum, labels,
			   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
			   3, cv::KMEANS_PP_CENTERS, centers);

	std::cout << "Compactness: " << compactness << std::endl;

	for (size_t i = 0; i < clusterNum; ++i)
	{
		memcpy(clusterCenters[i].data(), centers.data + i * descriptorSize * sizeof(float), sizeof(float) * descriptorSize);
	}

	for (size_t i = 0; i < keyptReprs.size(); ++i)
	{
		assignKeypointsToClusters(keyptReprs[i]);
	}
}

void VisualWord::assignKeypointsToClusters(KeypointRepresentation & keyptRepr) const
{
	if (clusterCenters.size() != clusterNum)
	{
		std::cerr << "Error: no clustering after reset cluster num!" << std::endl;
		return;
	}
	size_t num = 0;
	if (clusterAssignmentMode == HARD_ASSIGNMENT)
		num = 1;
	else if (clusterAssignmentMode == SOFT_ASSIGNMENT)
		num = 3;
	else
		std::cerr <<  "Error: invalid assignment mode!" << std::endl;

	std::vector<int> clusterLabels;
	for (size_t i = 0; i < keyptRepr.getNum(); ++i)
	{
		std::vector<int> indexes = findCloseClusterCenter(keyptRepr.getDescriptors() + i * KeypointRepresentation::descriptorSize, num);
		clusterLabels.insert(clusterLabels.end(), indexes.begin(), indexes.end());
	}

	keyptRepr.setClusterLabels(clusterLabels, clusterNum);
}

std::vector<int>  VisualWord::findCloseClusterCenter(const float * descriptor, size_t num) const
{
	std::vector<int> closeClusterIndex;
	std::list<std::pair<double, int>> closeClusterDisAndIndex;
	std::list<std::pair<double, int>> tempList;

	if (num == 0)
	{
		return closeClusterIndex;
	}

	for (size_t i = 0; i < clusterNum; ++i)
	{
		double distance = 0;
		for (int j = 0; j < KeypointRepresentation::descriptorSize; ++j)
		{
			distance += pow(descriptor[j] - clusterCenters[i][j], 2);
		}
		distance = std::sqrt(distance);

		tempList.push_back(std::make_pair(distance, i));

		closeClusterDisAndIndex.merge(tempList,
		[](std::pair<double, int> p1, std::pair<double, int> p2)
		{
			return p1.first < p2.first;		
		}
		);

		if (closeClusterDisAndIndex.size() > num)
			closeClusterDisAndIndex.pop_back();
	}

	for (const auto & index : closeClusterDisAndIndex)
	{
		closeClusterIndex.push_back(index.second);
	}
	return closeClusterIndex;
}

void VisualWord::load(const std::string & filePath)
{
	std::ifstream inputFile(filePath.c_str());
	if (!inputFile)
	{
		std::cerr << "Error: could not open visual words file!" << std::endl;
		return;
	}

	inputFile >> *this;
	inputFile.close();
}

void VisualWord::save(const std::string & filePath)
{
	std::ofstream outputFile(filePath.c_str());
	if (!outputFile)
	{
		std::cerr << "Error: could not open visual words file!" << std::endl;
		return;
	}

	outputFile << *this;
	outputFile.close();
}

std::istream & operator>>(std::istream & is, VisualWord & visualWord)
{
	is >> visualWord.clusterNum;
	is >> visualWord.clusterAssignmentMode;
	
	// Load in cluster centers
	visualWord.clusterCenters.resize(visualWord.clusterNum);
	for (size_t i = 0; i < visualWord.clusterNum; ++i)
	{
		for (int j = 0; j < KeypointRepresentation::descriptorSize; ++j)
		{
			is >> visualWord.clusterCenters[i][j];
		}
	}

	return is;
}

std::ostream & operator<<(std::ostream & os, const VisualWord & visualWord)
{
	os << visualWord.clusterNum << " " << visualWord.clusterAssignmentMode << std::endl;

	for (size_t i = 0; i < visualWord.clusterNum; ++i)
	{
		for (int j = 0; j < KeypointRepresentation::descriptorSize; ++j)
		{
			os << visualWord.clusterCenters[i][j] << " ";
		}
		os << std::endl;
	}

	return os;
}
