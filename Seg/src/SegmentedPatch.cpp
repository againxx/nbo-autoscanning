#include "SegmentedPatch.h"

SegmentedPatch::SegmentedPatch(int num, Eigen::Vector3f * positions, Eigen::Vector3f * colors, Eigen::Vector3f * normals, float * radii, unsigned short label)
 : pointNum(num),
   pointPositions(positions, [](Eigen::Vector3f * p){delete [] p;}),
   pointColors(colors, [](Eigen::Vector3f * p){delete [] p;}),
   pointNormals(normals, [](Eigen::Vector3f * p){delete [] p;}),
   pointRadii(radii, [](float * p){delete [] p;}),
   labelIndexes(1, label)
{
	denoise();
	
	//Compute bounding box, center position 
	centerPosition.setZero();
	boundingBox.reset(pointPositions.get(), pointNum);

	for(int i = 0; i < pointNum; ++i)
	{
		centerPosition += pointPositions.get()[i];	
	}
	centerPosition /= pointNum;
}

SegmentedPatch::SegmentedPatch(int num, const Eigen::Vector4f * mapData, unsigned short label, float confidence)
{
	pointNum = 0;
	for(int i = 0; i < num; ++i)
	{
		Eigen::Vector4f pos = mapData[3 * i];

		if(pos[3] > confidence)
		{
			Eigen::Vector4f col = mapData[3 * i + 1];	
			Eigen::Vector4f nor = mapData[3 * i + 2];
			if(col[1] == label && !isnan(nor[0]))
				pointNum++;
		}
	}

	pointPositions.reset(new Eigen::Vector3f[pointNum], [](Eigen::Vector3f * p){delete [] p;});
	pointColors.reset(new Eigen::Vector3f[pointNum], [](Eigen::Vector3f * p){delete [] p;});
	pointNormals.reset(new Eigen::Vector3f[pointNum], [](Eigen::Vector3f * p){delete [] p;});
	pointRadii.reset(new float[pointNum], [](float * p){delete [] p;});

	int count = 0;
	for(int i = 0; i < num; ++i)
	{
		Eigen::Vector4f pos = mapData[3 * i];

		if(pos[3] > confidence)
		{
			Eigen::Vector4f col = mapData[3 * i + 1];
			Eigen::Vector4f nor = mapData[3 * i + 2];
			if(col[1] == label && !isnan(nor[0]))
			{
				for(int j = 0; j < 3; ++j)
				{
					pointPositions.get()[count][j] = pos[j];
					pointNormals.get()[count][j] = -nor[j];
				}
				pointColors.get()[count][0] = static_cast<int>(col[0]) >> 16 & 0xFF;
				pointColors.get()[count][1] = static_cast<int>(col[0]) >> 8 & 0xFF;
				pointColors.get()[count][2] = static_cast<int>(col[0]) & 0xFF;
				pointRadii.get()[count] = nor[3];
				count++;
			}
		}
	}
	
	denoise();

	//Compute Bounding box, Center position 
	centerPosition.setZero();
	boundingBox.reset(pointPositions.get(), pointNum);

	for(int i = 0; i < pointNum; ++i)
	{
		centerPosition += pointPositions.get()[i];	
	}
	centerPosition /= pointNum;

	labelIndexes.push_back(label);
}

SegmentedPatch::SegmentedPatch(const std::string & filePath)
{
	load(filePath);
}

SegmentedPatch::~SegmentedPatch()
{

}

void SegmentedPatch::denoise()
{
	if (pointNum)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// Copy point position to pointcloud
		cloud->width = pointNum;
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);

		for (int i = 0; i < pointNum; ++i)
		{
			cloud->points[i].x = pointPositions.get()[i][0];
			cloud->points[i].y = pointPositions.get()[i][1];
			cloud->points[i].z = pointPositions.get()[i][2];
		}

		// Create the filtering object
		std::vector<int> filterIndex;
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(3);
		sor.filter(filterIndex);
	
		Eigen::Vector3f * positions = nullptr;
		Eigen::Vector3f * colors = nullptr;
		Eigen::Vector3f * normals = nullptr;
		float * radii = nullptr;

		int num = filterIndex.size();
		positions = new Eigen::Vector3f[num];
		if (pointColors)
			colors = new Eigen::Vector3f[num];
		if (pointNormals)
			normals = new Eigen::Vector3f[num];
		if (pointRadii)
			radii = new float[num];

		for (int i = 0; i < num; ++i)
		{
			positions[i] = pointPositions.get()[filterIndex[i]];
			if (pointColors)
				colors[i] = pointColors.get()[filterIndex[i]];
			if (pointNormals)
				normals[i] = pointNormals.get()[filterIndex[i]];
			if (pointRadii)
				radii[i] = pointRadii.get()[filterIndex[i]];
		}
		setPointData(num, positions, colors, normals, radii);
	}
}

void SegmentedPatch::save(const std::string & filePath)
{
	PlyWriter plyWriter(filePath);
	plyWriter.write(pointNum,
					pointPositions.get(),
					pointColors ? pointColors.get() : nullptr,
					pointNormals ? pointNormals.get() : nullptr,
					pointRadii ? pointRadii.get() : nullptr,
					true);
}

void SegmentedPatch::load(const std::string & filePath)
{
	PlyReader plyReader(filePath);

	Eigen::Vector3f * positions = nullptr;
	Eigen::Vector3f * colors = nullptr;
	Eigen::Vector3f * normals = nullptr;
	float * radii = nullptr;

	if (!plyReader.read(pointNum, positions, colors, normals, radii))
	{
		std::cerr << "Error: fail to load .ply file." << std::endl;
		return;
	}

	positions ? pointPositions.reset(positions, [](Eigen::Vector3f * p){delete [] p;}) : pointPositions.reset();
	colors ? pointColors.reset(colors, [](Eigen::Vector3f * p){delete [] p;}) : pointColors.reset();
	normals ? pointNormals.reset(normals, [](Eigen::Vector3f * p){delete [] p;}) : pointNormals.reset();
	radii ? pointRadii.reset(radii, [](float * p){delete [] p;}) : pointRadii.reset();

	denoise();

	//Compute Bounding box, Center position 
	centerPosition.setZero();
	boundingBox.reset(pointPositions.get(), pointNum);
	for(int i = 0; i < pointNum; ++i)
	{
		centerPosition += pointPositions.get()[i];	
	}
	centerPosition /= pointNum;
}

void SegmentedPatch::saveLocalPly(const Eigen::Vector3f & keypoint, const std::string & filePath)
{
	std::vector<Eigen::Vector3f> neighbourPositions;
	std::vector<Eigen::Vector3f> neighbourNormals;
	std::vector<Eigen::Vector3f> neighbourColors;

	for (int i = 0; i < pointNum; ++i)
	{
		if (std::abs(pointPositions.get()[i][0] - keypoint[0]) <= 0.3 && 
			std::abs(pointPositions.get()[i][1] - keypoint[1]) <= 0.3 &&
			std::abs(pointPositions.get()[i][2] - keypoint[2]) <= 0.3)
		{
			neighbourPositions.push_back(pointPositions.get()[i]);
			neighbourNormals.push_back(pointNormals.get()[i]);
			if (std::abs(pointPositions.get()[i][0] - keypoint[0]) <= 0.05 && 
					std::abs(pointPositions.get()[i][1] - keypoint[1]) <= 0.05 &&
					std::abs(pointPositions.get()[i][2] - keypoint[2]) <= 0.05)
			{
				neighbourColors.emplace_back(255, 0, 0);
			}
			else
				neighbourColors.emplace_back(180, 180, 180);

		}
	}

	PlyWriter plyWriter(filePath);
	plyWriter.write(neighbourPositions.size(), neighbourPositions.data(), neighbourColors.data(), neighbourNormals.data(), nullptr);
}

SegmentedPatch & SegmentedPatch::operator+=(const SegmentedPatch & otherPatch)
{
	if (this->pointNum == 0)
	{
		*this = otherPatch;
		return *this;
	}
	if (otherPatch.pointNum == 0)
	{
		return *this;
	}

	Eigen::Vector3f * positions = nullptr;
	Eigen::Vector3f * colors = nullptr;
	Eigen::Vector3f * normals = nullptr;
	float * radii = nullptr;
	int num = this->pointNum + otherPatch.pointNum;

	positions = new Eigen::Vector3f[num];
	memcpy(positions, this->pointPositions.get(), sizeof(Eigen::Vector3f) * this->pointNum);
	memcpy(positions + this->pointNum, otherPatch.pointPositions.get(), sizeof(Eigen::Vector3f) * otherPatch.pointNum);

	if (this->hasColor() && otherPatch.hasColor())
	{
		colors = new Eigen::Vector3f[num];
		memcpy(colors, this->pointColors.get(), sizeof(Eigen::Vector3f) * this->pointNum);
		memcpy(colors + this->pointNum, otherPatch.pointColors.get(), sizeof(Eigen::Vector3f) * otherPatch.pointNum);
	}
	if (this->hasNormal() && otherPatch.hasNormal())
	{
		normals = new Eigen::Vector3f[num];
		memcpy(normals, this->pointNormals.get(), sizeof(Eigen::Vector3f) * this->pointNum);
		memcpy(normals + this->pointNum, otherPatch.pointNormals.get(), sizeof(Eigen::Vector3f) * otherPatch.pointNum);
	}
	if (this->hasRadius() && otherPatch.hasRadius())
	{
		radii = new float[num];
		memcpy(radii, this->pointRadii.get(), sizeof(float) * this->pointNum);
		memcpy(radii + this->pointNum, otherPatch.pointRadii.get(), sizeof(float) * otherPatch.pointNum);
	}

	this->setBoundingBox(this->boundingBox + otherPatch.boundingBox);
	this->setCenterPosition((this->centerPosition * this->pointNum + otherPatch.centerPosition * otherPatch.pointNum) / num);
	this->setPointData(num, positions, colors, normals, radii);

	std::vector<unsigned short> indexes = this->labelIndexes;
	indexes.insert(indexes.end(), otherPatch.labelIndexes.begin(), otherPatch.labelIndexes.end());
	sort(indexes.begin(), indexes.end());
	setLabelIndexes(indexes);
	return *this;
}

SegmentedPatch & SegmentedPatch::operator*=(const Eigen::Matrix4f & transMat)
{
	Eigen::Matrix3f rotMat = transMat.topLeftCorner(3, 3);
	Eigen::Vector3f transVec = transMat.topRightCorner(3, 1);

	Eigen::Vector3f * positions = nullptr;
	Eigen::Vector3f * colors = nullptr;
	Eigen::Vector3f * normals = nullptr;
	float * radii = nullptr;

	positions = new Eigen::Vector3f[pointNum];
	memcpy(positions, pointPositions.get(), sizeof(Eigen::Vector3f) * pointNum);

	if (hasColor())
	{
		colors = new Eigen::Vector3f[pointNum];
		memcpy(colors, pointColors.get(), sizeof(Eigen::Vector3f) * pointNum);
	}
	if (hasNormal())
	{
		normals = new Eigen::Vector3f[pointNum];
		memcpy(normals, pointNormals.get(), sizeof(Eigen::Vector3f) * pointNum);
	}
	if (hasRadius())
	{
		radii = new float[pointNum];
		memcpy(radii, pointRadii.get(), sizeof(float) * pointNum);
	}

	for (int i = 0; i < pointNum; ++i)
	{
		positions[i] = rotMat * positions[i] + transVec;
	}

	if (hasNormal())
	{
		for (int i = 0; i < pointNum; ++i)
		{
			normals[i] = rotMat * normals[i];
		}
	}

	setPointData(pointNum, positions, colors, normals, radii);
	setCenterPosition(rotMat * centerPosition + transVec);
	boundingBox.reset(pointPositions.get(), pointNum);

	return *this;
}

void SegmentedPatch::setPointData(int num, Eigen::Vector3f * positions, Eigen::Vector3f * colors, Eigen::Vector3f * normals, float * radii)
{
	positions ? pointPositions.reset(positions, [](Eigen::Vector3f * p){delete [] p;}) : pointPositions.reset();
	colors ? pointColors.reset(colors, [](Eigen::Vector3f * p){delete [] p;}) : pointColors.reset();
	normals? pointNormals.reset(normals, [](Eigen::Vector3f * p){delete [] p;}) : pointNormals.reset();
	radii ? pointRadii.reset(radii, [](float * p){delete [] p;}) : pointRadii.reset();
	pointNum= num;
}

SegmentedPatch operator+(const SegmentedPatch & lPatch, const SegmentedPatch & rPatch)
{
	SegmentedPatch newPatch = lPatch;
	newPatch += rPatch;
	return newPatch;
}

SegmentedPatch operator*(const SegmentedPatch & patch, const Eigen::Matrix4f & transMat)
{
	SegmentedPatch newPatch = patch;
	newPatch *= transMat;
	return newPatch;
}

bool isAdjacent(const SegmentedPatch & lPatch, const SegmentedPatch & rPatch)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud1(new pcl::PointCloud<pcl::PointXYZ>);

	pointCloud1->width = lPatch.getPointNum();
	pointCloud1->height = 1;
	pointCloud1->points.resize(pointCloud1->width * pointCloud1->height);

	// Copy point data from SegmentedPatch to pcl
	for (int i = 0; i < lPatch.getPointNum(); ++i)
	{
		pointCloud1->points[i].x = lPatch.getPointPositions()[i][0];
		pointCloud1->points[i].y = lPatch.getPointPositions()[i][1];
		pointCloud1->points[i].z = lPatch.getPointPositions()[i][2];
	}

	// Generate kd-tree
	pcl::KdTreeFLANN<pcl::PointXYZ> KDTree1;

	KDTree1.setInputCloud(pointCloud1);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = SegmentedPatch::paraThresholdClose;
	int closeCount = 0;
	for (int i = 0; i < rPatch.getPointNum(); ++i)
	{
		pcl::PointXYZ searchPoint;
		searchPoint.x = rPatch.getPointPositions()[i][0];
		searchPoint.y = rPatch.getPointPositions()[i][1];
		searchPoint.z = rPatch.getPointPositions()[i][2];

		// Neighbors within radius search
		if (KDTree1.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			if (pointIdxRadiusSearch.size() > 5)
				closeCount++;
		}
		if (closeCount > 5)
			return true;
	}
	return false;
}
