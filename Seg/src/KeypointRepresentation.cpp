#include "KeypointRepresentation.h"

KeypointRepresentation::KeypointRepresentation(unsigned int keyptNum,
											   std::shared_ptr<Eigen::Vector3f> keyptPos,
											   std::shared_ptr<Eigen::Vector3f> keyptNor,
											   std::shared_ptr<float> keyptDesc,
											   std::shared_ptr<std::set<std::pair<int, int>>> keyptNeigh,
											   const Eigen::Vector3f & center,
											   float pcDiameter,
											   const std::string & modelName)
 : name(modelName),
   num(keyptNum),
   positions(keyptPos),
   normals(keyptNor),
   descriptors(keyptDesc),
   neighborhoods(keyptNeigh),
   centerPosition(center),
   diameter(pcDiameter)
{
	computeAverageRadius();
	computeAverageLength();
}

KeypointRepresentation::~KeypointRepresentation()
{

}

void KeypointRepresentation::computeAverageRadius()
{
	double tempR = 0;
	for (size_t i = 0; i < num; ++i)
	{
		tempR += (positions.get()[i] - centerPosition).norm();	
	}
	averageRadius = tempR / num;
}

void KeypointRepresentation::computeAverageLength()
{
	double tempL = 0;
	for (size_t i = 0; i < num; ++i)
	{
		for (size_t j = i + 1; j < num; ++j)
		{
			tempL += (positions.get()[i] - positions.get()[j]).norm();
		}
	}
	averageLength = tempL / (0.5 * num * (num - 1));
}

void KeypointRepresentation::computeClusterLabelHistogram()
{
	clusterLabelHistogram.resize(clusterNum);
	clusterLabelHistogram.setZero();
	for (auto label : clusterLabels)
	{
		clusterLabelHistogram[label] += 1.0;
	}

	clusterLabelHistogram /= clusterLabels.size();
}

void KeypointRepresentation::computeNeighborLabelHistogram()
{
	neighborLabelHistogram.resize(clusterNum, clusterNum);
	neighborLabelHistogram.setZero();
	
	int assignmentNum = clusterLabels.size() / num;
	for (size_t i = 0; i < num; ++i)
	{
		Eigen::VectorXd indicativeVector1(clusterNum);
		indicativeVector1.setZero();

		for (int j = 0; j < assignmentNum; ++j)
		{
			indicativeVector1[clusterLabels[i * assignmentNum + j]] = 1;
		}

		for (size_t j = 0; j < num; ++j)
		{
			if (isNeighbor(i, j))
			{
				Eigen::VectorXd indicativeVector2(clusterNum);
				indicativeVector2.setZero();
					
				for (int k = 0; k < assignmentNum; ++k)
				{
					indicativeVector2[clusterLabels[j * assignmentNum + k]] = 1;
				}

				neighborLabelHistogram += indicativeVector1 * indicativeVector2.transpose();
			}
		}
	}
	/* TODO: assignmentNum^2 * num or assignmentNum * num <31-08-17, xixia> */
	neighborLabelHistogram /= clusterLabels.size();
}

double KeypointRepresentation::computeHistogramDistance(const KeypointRepresentation & otherKeyptRepr) const
{
	if (otherKeyptRepr.getClusterLabelHistogram().size() != clusterLabelHistogram.size())
	{
		std::cerr << "Error: incomparable histogram size!" << std::endl;
		return -1;
	}

	double distance1 = 0;
	for (int i = 0; i < clusterLabelHistogram.size(); ++i)
	{
		distance1 += std::abs(otherKeyptRepr.getClusterLabelHistogram()[i] - clusterLabelHistogram[i]);
	}

	double distance2 = 0;
	for (int i = 0; i < neighborLabelHistogram.cols(); ++i)
	{
		double colDistance = 0;
		for (int j = 0; j < neighborLabelHistogram.rows(); ++j)
		{
			colDistance += std::abs(otherKeyptRepr.getNeighborLabelHistogram()(j, i) - neighborLabelHistogram(j, i));
		}
		distance2 = std::max(distance2, colDistance);
	}

	return 6 * distance1 + distance2;
}

void KeypointRepresentation::reset()
{
	positions.reset();
	normals.reset();
	descriptors.reset();
	neighborhoods.reset();
	num = 0;
	centerPosition = Eigen::Vector3f{0, 0, 0};
	averageRadius = 0;
	averageLength = 0;
	diameter = 0;
}

void KeypointRepresentation::load(const std::string & filePath)
{
	std::cout << "Loading " << filePath << std::endl;
	std::ifstream inputFile(filePath.c_str());

	try
	{
		if (!inputFile)
		{
			throw std::runtime_error(filePath + "\nError: could not open keypoint file!");
		}

		inputFile >> *this;
	}
	catch (std::runtime_error err)
	{
		std::cerr << err.what() << std::endl;
		num = 0;
	}

	inputFile.close();
}

std::istream & operator>>(std::istream & is, KeypointRepresentation & keyptRepr)
{
	is >> keyptRepr.num;
	
	// Load in keypoint positions and normals
	keyptRepr.positions.reset(new Eigen::Vector3f[keyptRepr.num], [](Eigen::Vector3f * p){delete [] p;});
	keyptRepr.normals.reset(new Eigen::Vector3f[keyptRepr.num], [](Eigen::Vector3f * p){delete [] p;});
	for (size_t i = 0; i < keyptRepr.num; ++i)
	{
		is >> keyptRepr.positions.get()[i][0];
		is >> keyptRepr.positions.get()[i][1];
		is >> keyptRepr.positions.get()[i][2];

		is >> keyptRepr.normals.get()[i][0];
		is >> keyptRepr.normals.get()[i][1];
		is >> keyptRepr.normals.get()[i][2];
	}

	// Load in center point, average radius, average length and diameter
	is >> keyptRepr.centerPosition[0];
	is >> keyptRepr.centerPosition[1];
	is >> keyptRepr.centerPosition[2];

	is >> keyptRepr.averageRadius;
	is >> keyptRepr.averageLength;
	is >> keyptRepr.diameter;

	// Load in keypoint descriptors
	int tempSize;
	is >> tempSize;
	if (tempSize != keyptRepr.descriptorSize)
	{
		throw std::runtime_error("Error: descriptor size isn't matched!");
	}
	keyptRepr.descriptors.reset(new float[keyptRepr.num * keyptRepr.descriptorSize], [](float * p){delete [] p;});
	for (size_t i = 0; i < keyptRepr.num; ++i)
	{
		for (int j = 0; j < keyptRepr.descriptorSize; ++j)
		{
			is >> keyptRepr.descriptors.get()[i * keyptRepr.descriptorSize + j];
		}
	}

	keyptRepr.neighborhoods.reset(new std::set<std::pair<int, int>>);
	std::pair<int, int> tempPair;
	while (is >> tempPair.first >> tempPair.second)
		keyptRepr.neighborhoods->insert(std::move(tempPair));

	return is;
}

std::ostream & operator<<(std::ostream & os, const KeypointRepresentation & keyptRepr)
{
	os << keyptRepr.num << std::endl;
	
	// Output keypoint positions and normals
	for (size_t i = 0; i < keyptRepr.num; ++i)
	{
		os << keyptRepr.positions.get()[i][0] << " ";
		os << keyptRepr.positions.get()[i][1] << " ";
		os << keyptRepr.positions.get()[i][2] << " ";

		os << keyptRepr.normals.get()[i][0] << " ";
		os << keyptRepr.normals.get()[i][1] << " ";
		os << keyptRepr.normals.get()[i][2] << std::endl;
	}

	// Output center point, average radius, average length and diameter
	os << keyptRepr.centerPosition[0] << " ";
	os << keyptRepr.centerPosition[1] << " ";
	os << keyptRepr.centerPosition[2] << " ";

	os << keyptRepr.averageRadius << " ";
	os << keyptRepr.averageLength << " ";
	os << keyptRepr.diameter << std::endl;

	//Output keypoint descriptor
	os << keyptRepr.descriptorSize << std::endl;
	for (size_t i = 0; i < keyptRepr.num; ++i)
	{
		for (int j = 0; j < keyptRepr.descriptorSize; ++j)
		{
			os << keyptRepr.descriptors.get()[i * keyptRepr.descriptorSize + j] << " ";
		}
		os << std::endl;
	}

	for (const auto & neighborPair : *keyptRepr.neighborhoods)
	{
		os << neighborPair.first << " " << neighborPair.second << std::endl;
	}

	return os;
}

void KeypointRepresentation::setClusterLabels(const std::vector<int> & labels, int num)
{
	clusterLabels = labels;
	clusterNum = num;
	computeClusterLabelHistogram();

	if (hasNeighborhoods())
		computeNeighborLabelHistogram();
}
