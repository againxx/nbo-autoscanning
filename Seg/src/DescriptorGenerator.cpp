#include "DescriptorGenerator.h"
#include "Utils/utils.hpp"

const int DescriptorGenerator::RANDOM_MODE = 0;
const int DescriptorGenerator::UNIFORM_MODE = 1;

DescriptorGenerator::DescriptorGenerator(const SegmentedPatch & patch)
{
	setPointData(patch);
}

DescriptorGenerator::DescriptorGenerator(const std::string & filePath)
{
	setPointData(filePath);
}


DescriptorGenerator::~DescriptorGenerator()
{
	if (pointPositions)
		delete [] pointPositions;
	if (pointNormals)
		delete [] pointNormals;
	if (voxelGridTDF)
		delete [] voxelGridTDF;
	if (keypointGrid)
		delete [] keypointGrid;
	if (adjacentPointPositions)
		delete [] adjacentPointPositions;
	if (adjacentPointNormals)
		delete [] adjacentPointNormals;
}

void DescriptorGenerator::setPointData(const SegmentedPatch & patch)
{
	if (pointPositions)
		delete [] pointPositions;
	if (pointNormals)
		delete [] pointNormals;
	if (adjacentPointPositions)
		delete [] adjacentPointPositions;
	if (adjacentPointNormals)
		delete [] adjacentPointNormals;

	pointNum = patch.getPointNum();
	pointPositions = new float[pointNum * 3];
	pointNormals = new float[pointNum * 3];
	centerPosition = patch.getCenterPosition();
	boundingBox = patch.getBoundingBox();
	diameter = boundingBox.getDiameter();

	adjacentPointNum = 0;
	adjacentPointPositions = nullptr;
	adjacentPointNormals = nullptr;
	adjacentBoundingBox.reset();

	// Set voxelSize according to the bounding box of the patch
	if (diameter > 1.0)
	{
		voxelSize = 0.02;
	}
	else
	{
		voxelSize = 0.01;
	}

	for (int i = 0; i < pointNum; ++i)
	{
		pointPositions[3 * i] = patch.getPointPositions()[i][0];
		pointPositions[3 * i + 1] = patch.getPointPositions()[i][1];
		pointPositions[3 * i + 2] = patch.getPointPositions()[i][2];

		pointNormals[3 * i] = patch.getPointNormals()[i][0];
		pointNormals[3 * i + 1] = patch.getPointNormals()[i][1];
		pointNormals[3 * i + 2] = patch.getPointNormals()[i][2];
	}
	isConsiderAdjacent = false;
}

void DescriptorGenerator::setPointData(const SegmentedPatch & patch, const SegmentedPatch & adjacentPatch)
{
	if (pointPositions)
		delete [] pointPositions;
	if (pointNormals)
		delete [] pointNormals;
	if (adjacentPointPositions)
		delete [] adjacentPointPositions;
	if (adjacentPointNormals)
		delete [] adjacentPointNormals;

	pointNum = patch.getPointNum();
	pointPositions = new float[pointNum * 3];
	pointNormals = new float[pointNum * 3];
	centerPosition = patch.getCenterPosition();
	boundingBox = patch.getBoundingBox();
	diameter = boundingBox.getDiameter();

	adjacentPointNum = adjacentPatch.getPointNum();
	adjacentPointPositions = new float[adjacentPointNum * 3];
	adjacentPointNormals = new float[adjacentPointNum * 3];
	adjacentBoundingBox = adjacentPatch.getBoundingBox();

	// Set voxelSize according to the bounding box of the patch
	if (adjacentBoundingBox.getDiameter() > 1.0)
	{
		voxelSize = 0.02;
	}
	else
	{
		voxelSize = 0.01;
	}

	for (int i = 0; i < pointNum; ++i)
	{
		pointPositions[3 * i] = patch.getPointPositions()[i][0];
		pointPositions[3 * i + 1] = patch.getPointPositions()[i][1];
		pointPositions[3 * i + 2] = patch.getPointPositions()[i][2];

		pointNormals[3 * i] = patch.getPointNormals()[i][0];
		pointNormals[3 * i + 1] = patch.getPointNormals()[i][1];
		pointNormals[3 * i + 2] = patch.getPointNormals()[i][2];
	}

	for (int i = 0; i < adjacentPointNum; ++i)
	{
		adjacentPointPositions[3 * i] = adjacentPatch.getPointPositions()[i][0];
		adjacentPointPositions[3 * i + 1] = adjacentPatch.getPointPositions()[i][1];
		adjacentPointPositions[3 * i + 2] = adjacentPatch.getPointPositions()[i][2];

		adjacentPointNormals[3 * i] = adjacentPatch.getPointNormals()[i][0];
		adjacentPointNormals[3 * i + 1] = adjacentPatch.getPointNormals()[i][1];
		adjacentPointNormals[3 * i + 2] = adjacentPatch.getPointNormals()[i][2];
	}
	isConsiderAdjacent = true;
}

void DescriptorGenerator::setPointData(const std::string & filePath)
{
	SegmentedPatch patch(filePath);
	setPointData(patch);
}

KeypointRepresentation DescriptorGenerator::computeDescriptor(int keyptNum)
{
	if (voxelGridTDF)
		delete [] voxelGridTDF;
	if (keypointGrid)
		delete [] keypointGrid;

	keypointNum = keyptNum;
	computeTDF();

	if (samplingMode == RANDOM_MODE)
		randomlySampleKeypoints();
	else if (samplingMode == UNIFORM_MODE)
		uniformlySampleKeypoints();

	keypointDescriptors.reset(new float[keypointNum * descriptorSize], [](float * p){delete [] p;});
	auto startTime = std::chrono::system_clock::now();
	compute3DMatchDescriptor(keypointNum, batchSize, descriptorSize, keypointDescriptors.get(), keypointGrid, voxelGridTDF, voxelGridDimX, voxelGridDimY, voxelGridDimZ);
	auto endTime = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	//std::cout << "Spend " << (double)duration.count() * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den  << " seconds computing descriptor." << std::endl;

	KeypointRepresentation keyptRepr(keypointNum, keypointPositions, keypointNormals, keypointDescriptors, keypointNeighborhoods, centerPosition, diameter);
	return keyptRepr;
}

void DescriptorGenerator::computeTDF()
{
	//std::cout << "Current Voxel Size is: " << voxelSize << std::endl;
	if (!isConsiderAdjacent)
	{
		// Compute point cloud coordinates of the origin voxel (0,0,0) of the voxel grid
		voxelGridOriginX = boundingBox.xMin;
		voxelGridOriginY = boundingBox.yMin;
		voxelGridOriginZ = boundingBox.zMin;

		voxelGridMaxX = boundingBox.xMax;
		voxelGridMaxY = boundingBox.yMax;
		voxelGridMaxZ = boundingBox.zMax;

		voxelGridDimX = round((voxelGridMaxX - voxelGridOriginX) / voxelSize) + 1 + voxelGridPadding * 2;
		voxelGridDimY = round((voxelGridMaxY - voxelGridOriginY) / voxelSize) + 1 + voxelGridPadding * 2;
		voxelGridDimZ = round((voxelGridMaxZ - voxelGridOriginZ) / voxelSize) + 1 + voxelGridPadding * 2;

		voxelGridOriginX = voxelGridOriginX - voxelGridPadding * voxelSize + voxelSize / 2;
		voxelGridOriginY = voxelGridOriginY - voxelGridPadding * voxelSize + voxelSize / 2;
		voxelGridOriginZ = voxelGridOriginZ - voxelGridPadding * voxelSize + voxelSize / 2;

		//std::cout << "Size of TDF voxel grid: " << voxelGridDimX << " x " << voxelGridDimY << " x " << voxelGridDimZ << std::endl;
		//std::cout << "Computing TDF voxel grid..." << std::endl;

		// Compute surface occupancy grid
		float * voxelGridOcc = new float[voxelGridDimX * voxelGridDimY * voxelGridDimZ];
		memset(voxelGridOcc, 0, sizeof(float) * voxelGridDimX * voxelGridDimY * voxelGridDimZ);
		for (int i = 0 ; i < pointNum; ++i)
		{
			int pointGridX = round((pointPositions[3 * i] - voxelGridOriginX) / voxelSize);
			int pointGridY = round((pointPositions[3 * i + 1] - voxelGridOriginY) / voxelSize);
			int pointGridZ = round((pointPositions[3 * i + 2] - voxelGridOriginZ) / voxelSize);
			voxelGridOcc[pointGridZ * voxelGridDimY * voxelGridDimX + pointGridY * voxelGridDimX + pointGridX] = 1.0f;
		}

		//Initialize TDF voxel grid
		voxelGridTDF = new float[voxelGridDimX * voxelGridDimY * voxelGridDimZ];
		memset(voxelGridTDF, 0, sizeof(float) * voxelGridDimX * voxelGridDimY * voxelGridDimZ);

		computeTDFVoxelGrid(voxelGridOcc, voxelGridTDF,
							voxelGridDimX, voxelGridDimY, voxelGridDimZ,
							voxelSize, truncMargin);

		delete [] voxelGridOcc;
	}
	else
	{
		voxelGridOriginX = adjacentBoundingBox.xMin;
		voxelGridOriginY = adjacentBoundingBox.yMin;
		voxelGridOriginZ = adjacentBoundingBox.zMin;

		voxelGridMaxX = adjacentBoundingBox.xMax;
		voxelGridMaxY = adjacentBoundingBox.yMax;
		voxelGridMaxZ = adjacentBoundingBox.zMax;

		voxelGridDimX = round((voxelGridMaxX - voxelGridOriginX) / voxelSize) + 1 + voxelGridPadding * 2;
		voxelGridDimY = round((voxelGridMaxY - voxelGridOriginY) / voxelSize) + 1 + voxelGridPadding * 2;
		voxelGridDimZ = round((voxelGridMaxZ - voxelGridOriginZ) / voxelSize) + 1 + voxelGridPadding * 2;

		voxelGridOriginX = voxelGridOriginX - voxelGridPadding * voxelSize + voxelSize / 2;
		voxelGridOriginY = voxelGridOriginY - voxelGridPadding * voxelSize + voxelSize / 2;
		voxelGridOriginZ = voxelGridOriginZ - voxelGridPadding * voxelSize + voxelSize / 2;

		//std::cout << "Size of TDF voxel grid: " << voxelGridDimX << " x " << voxelGridDimY << " x " << voxelGridDimZ << std::endl;
		//std::cout << "Computing TDF voxel grid..." << std::endl;

		// Compute surface occupancy grid
		float * voxelGridOcc = new float[voxelGridDimX * voxelGridDimY * voxelGridDimZ];
		memset(voxelGridOcc, 0, sizeof(float) * voxelGridDimX * voxelGridDimY * voxelGridDimZ);
		for (int i = 0 ; i < adjacentPointNum; ++i)
		{
			int pointGridX = round((adjacentPointPositions[3 * i] - voxelGridOriginX) / voxelSize);
			int pointGridY = round((adjacentPointPositions[3 * i + 1] - voxelGridOriginY) / voxelSize);
			int pointGridZ = round((adjacentPointPositions[3 * i + 2] - voxelGridOriginZ) / voxelSize);
			voxelGridOcc[pointGridZ * voxelGridDimY * voxelGridDimX + pointGridY * voxelGridDimX + pointGridX] = 1.0f;
		}

		//Initialize TDF voxel grid
		voxelGridTDF = new float[voxelGridDimX * voxelGridDimY * voxelGridDimZ];
		memset(voxelGridTDF, 0, sizeof(float) * voxelGridDimX * voxelGridDimY * voxelGridDimZ);

		computeTDFVoxelGrid(voxelGridOcc, voxelGridTDF,
							voxelGridDimX, voxelGridDimY, voxelGridDimZ,
							voxelSize, truncMargin);

		delete [] voxelGridOcc;
	}
}

// Compute random surface keypoints in point cloud coordinates and voxel grid coordinates
void DescriptorGenerator::randomlySampleKeypoints()
{
	//std::cout << "Finding random surface keypoints..." << std::endl;
	keypointPositions.reset(new Eigen::Vector3f[keypointNum], [](Eigen::Vector3f * p){delete [] p;});
	keypointNormals.reset(new Eigen::Vector3f[keypointNum], [](Eigen::Vector3f * p){delete [] p;});
	keypointGrid = new float[keypointNum * 3];
	keypointNeighborhoods.reset(new std::set<std::pair<int, int>>);

	for (int i = 0; i < keypointNum; ++i)
	{
		int randIDX = (int)(GetRandomFloat(0.0f, (float)pointNum));
		keypointPositions.get()[i][0] = pointPositions[3 * randIDX];
		keypointPositions.get()[i][1] = pointPositions[3 * randIDX + 1];
		keypointPositions.get()[i][2] = pointPositions[3 * randIDX + 2];

		keypointNormals.get()[i][0] = pointNormals[3 * randIDX];
		keypointNormals.get()[i][1] = pointNormals[3 * randIDX + 1];
		keypointNormals.get()[i][2] = pointNormals[3 * randIDX + 2];

		keypointGrid[3 * i] = round((pointPositions[3 * randIDX] - voxelGridOriginX) / voxelSize);
		keypointGrid[3 * i + 1] = round((pointPositions[3 * randIDX + 1] - voxelGridOriginY) / voxelSize);
		keypointGrid[3 * i + 2] = round((pointPositions[3 * randIDX + 2] - voxelGridOriginZ) / voxelSize);

		if (keypointGrid[3 * i] - 15 < 0)
			keypointGrid[3 * i]++;
		if (keypointGrid[3 * i + 1] - 15 < 0)
			keypointGrid[3 * i + 1]++;
		if (keypointGrid[3 * i + 2] - 15 < 0)
			keypointGrid[3 * i + 2]++;
		if (keypointGrid[3 * i] + 15 >= voxelGridDimX)
			keypointGrid[3 * i]--;
		if (keypointGrid[3 * i + 1] + 15 >= voxelGridDimY)
			keypointGrid[3 * i + 1]--;
		if (keypointGrid[3 * i + 2] + 15 >= voxelGridDimZ)
			keypointGrid[3 * i + 2]--;
	}
}

// Compute uniform surface keypoints in point cloud coordinates and voxel grid coordinates
void DescriptorGenerator::uniformlySampleKeypoints()
{
	//std::cout << "Finding uniform surface keypoints..." << std::endl;
	keypointPositions.reset(new Eigen::Vector3f[keypointNum], [](Eigen::Vector3f * p){delete [] p;});
	keypointNormals.reset(new Eigen::Vector3f[keypointNum], [](Eigen::Vector3f * p){delete [] p;});
	keypointGrid = new float[keypointNum * 3];
	keypointNeighborhoods.reset(new std::set<std::pair<int, int>>);

	std::unique_ptr<int[]> seedIndexes(new int[keypointNum]);
	std::unique_ptr<int[]> newSeedIndexes(new int[keypointNum]);
	for (int i = 0; i < keypointNum; ++i)
	{
		int randIndex = (int)(GetRandomFloat(0.0f, (float)pointNum));
		seedIndexes[i] = randIndex;
	}

	std::unique_ptr<int[]> pointLabels(new int[pointNum]);
	std::unique_ptr<int[]> neighborIndexes(new int[pointNum]);
	bool terminateFlag = false;

	int loopCount = 0;

	do
	{
		labelPointToClosestSeed(pointNum, keypointNum, pointPositions, seedIndexes.get(), pointLabels.get(), neighborIndexes.get());

		std::unique_ptr<double[]> centroids(new double[3 * keypointNum]);
		memset(centroids.get(), 0, sizeof(double) * 3 * keypointNum);
		std::unique_ptr<int[]> labeledPointCounts(new int[keypointNum]);
		memset(labeledPointCounts.get(), 0, sizeof(int) * keypointNum);

		for (int i = 0; i < pointNum; ++i)
		{
			centroids[3 * pointLabels[i]] += pointPositions[3 * i];
			centroids[3 * pointLabels[i] + 1] += pointPositions[3 * i + 1];
			centroids[3 * pointLabels[i] + 2] += pointPositions[3 * i + 2];
			++labeledPointCounts[pointLabels[i]];
		}

		std::unique_ptr<double[]> minDistanceToCentroids(new double[keypointNum]);
		for (int i = 0; i < keypointNum; ++i)
		{
			if (labeledPointCounts[i] > 0)
			{
				centroids[3 * i] /= labeledPointCounts[i];
				centroids[3 * i + 1] /= labeledPointCounts[i];
				centroids[3 * i + 2] /= labeledPointCounts[i];
			}
			minDistanceToCentroids[i] = std::numeric_limits<double>::max();
			newSeedIndexes[i] = -1;
		}

		for (int i = 0; i < pointNum; ++i)
		{
			double distance = sqrt(pow(pointPositions[3 * i] - centroids[3 * pointLabels[i]], 2)
								 + pow(pointPositions[3 * i + 1] - centroids[3 * pointLabels[i] + 1], 2)
								 + pow(pointPositions[3 * i + 2] - centroids[3 * pointLabels[i] + 2], 2));

			if (distance < minDistanceToCentroids[pointLabels[i]])
			{
				minDistanceToCentroids[pointLabels[i]] = distance;
				newSeedIndexes[pointLabels[i]] = i;
			}
		}

		double iterError = 0;
		for (int i = 0; i < keypointNum; ++i)
		{
			if (newSeedIndexes[i] != -1)
			{
				iterError += sqrt(pow(pointPositions[3 * seedIndexes[i]] - pointPositions[3 * newSeedIndexes[i]], 2)
								+ pow(pointPositions[3 * seedIndexes[i] + 1] - pointPositions[3 * newSeedIndexes[i] + 1], 2)
								+ pow(pointPositions[3 * seedIndexes[i] + 2] - pointPositions[3 * newSeedIndexes[i] + 2], 2));
				seedIndexes[i] = newSeedIndexes[i];
			}
		}
		iterError /= keypointNum;
		++loopCount;

		if (iterError < 0.001)
			terminateFlag = true;
	}
	while(!terminateFlag);

	for (int i = 0; i < keypointNum; ++i)
	{
		keypointPositions.get()[i][0] = pointPositions[3 * seedIndexes[i]];
		keypointPositions.get()[i][1] = pointPositions[3 * seedIndexes[i] + 1];
		keypointPositions.get()[i][2] = pointPositions[3 * seedIndexes[i] + 2];

		keypointNormals.get()[i][0] = pointNormals[3 * seedIndexes[i]];
		keypointNormals.get()[i][1] = pointNormals[3 * seedIndexes[i] + 1];
		keypointNormals.get()[i][2] = pointNormals[3 * seedIndexes[i] + 2];

		keypointGrid[3 * i] = round((pointPositions[3 * seedIndexes[i]] - voxelGridOriginX) / voxelSize);
		keypointGrid[3 * i + 1] = round((pointPositions[3 * seedIndexes[i] + 1] - voxelGridOriginY) / voxelSize);
		keypointGrid[3 * i + 2] = round((pointPositions[3 * seedIndexes[i] + 2] - voxelGridOriginZ) / voxelSize);

		if (keypointGrid[3 * i] - 15 < 0)
			keypointGrid[3 * i]++;
		if (keypointGrid[3 * i + 1] - 15 < 0)
			keypointGrid[3 * i + 1]++;
		if (keypointGrid[3 * i + 2] - 15 < 0)
			keypointGrid[3 * i + 2]++;
		if (keypointGrid[3 * i] + 15 >= voxelGridDimX)
			keypointGrid[3 * i]--;
		if (keypointGrid[3 * i + 1] + 15 >= voxelGridDimY)
			keypointGrid[3 * i + 1]--;
		if (keypointGrid[3 * i + 2] + 15 >= voxelGridDimZ)
			keypointGrid[3 * i + 2]--;
	}

	for (int i = 0; i < pointNum; ++i)
	{
		keypointNeighborhoods->emplace(pointLabels[i], neighborIndexes[i]);
	}
}

void DescriptorGenerator::saveTDF(const std::string & prefixFileName)
{
	std::cout << "Saving TDF to disk (.tdf)..." << std::endl;
	std::string tdfSavePath = prefixFileName + ".tdf";
	std::ofstream tdfOutFile(tdfSavePath);
	for (int i = 0; i < voxelGridDimX * voxelGridDimY * voxelGridDimZ; ++i)
	{
		tdfOutFile << voxelGridTDF[i] << " ";
		if (i % 30 == 0)
			tdfOutFile << std::endl;
	}
	tdfOutFile.close();
}

void DescriptorGenerator::saveKeypointLocalTDFs(const std::string & prefixFileName)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr localTDFCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	for (int i = 0; i < keypointNum; ++i)
	{
		localTDFCloud->clear();

		float keyptGridX = keypointGrid[3 * i];
		float keyptGridY = keypointGrid[3 * i + 1];
		float keyptGridZ = keypointGrid[3 * i + 2];
		for (int z = keyptGridZ - 15; z < keyptGridZ + 15; ++z)
		{
			for (int y = keyptGridY - 15; y < keyptGridY + 15; ++y)
			{
				for (int x = keyptGridX - 15; x < keyptGridX + 15; ++x)
				{
					if (voxelGridTDF[z * voxelGridDimY * voxelGridDimX + y * voxelGridDimX + x] > 0)
					{
						pcl::PointXYZRGBA point;
						point.x = x * voxelSize;
						point.y = y * voxelSize;
						point.z = z * voxelSize;
						point.r = 255 - 255 * (1.0f - voxelGridTDF[z * voxelGridDimY * voxelGridDimX + y * voxelGridDimX + x]);
						point.g = 0;
						point.b = 255 * (1.0f - voxelGridTDF[z * voxelGridDimY * voxelGridDimX + y * voxelGridDimX + x]);
						point.a = 1;
						localTDFCloud->points.push_back(point);
					
					}
				}
			}
		}
		pcl::io::savePLYFileBinary(prefixFileName + "/" + std::to_string(i) + ".ply", *localTDFCloud);
	}
}

void DescriptorGenerator::loadKeypoints(const std::string prefixFileName)
{
	std::string rkLoadPath = prefixFileName + ".rk";
	std::ifstream rkInFile(rkLoadPath);
	keypointPositions.reset(new Eigen::Vector3f[keypointNum], [](Eigen::Vector3f * p){delete [] p;});
	keypointGrid = new float[keypointNum * 3];

	for (int i = 0; i < keypointNum; ++i)
	{
		rkInFile >> keypointPositions.get()[i][0];
		rkInFile >> keypointPositions.get()[i][1];
		rkInFile >> keypointPositions.get()[i][2];

		rkInFile >> keypointGrid[3 * i];
		rkInFile >> keypointGrid[3 * i + 1];
		rkInFile >> keypointGrid[3 * i + 2];
	}
	rkInFile.close();
}

void DescriptorGenerator::setSamplingMode(int mode)
{
	if (mode != RANDOM_MODE && mode != UNIFORM_MODE)
	{
		std::cerr << "Error: invalid sampling mode!" << std::endl;
		return;
	}
	else
	{
		samplingMode = mode;
	}
}
