#include "MatchingResultParser.h"

MatchingResultParser::MatchingResultParser(const std::vector<Match> & matches, const std::vector<KeypointRepresentation> & models, const KeypointRepresentation & patch)
{
	for (const auto & match : matches)
	{
		modelNames.push_back(models[match.modelIndex].getName());

		Eigen::Vector3f * patchMatchedKeyptPos = new Eigen::Vector3f[match.pairNum];
		Eigen::Vector3f * modelMatchedKeyptPos = new Eigen::Vector3f[match.pairNum];

		for (int i = 0; i < match.pairNum; ++i)
		{
			patchMatchedKeyptPos[i] = patch.getPositions()[match.patchKeypointIndexes[i]];
			modelMatchedKeyptPos[i] = models[match.modelIndex].getPositions()[match.modelKeypointIndexes[i]];
		}


		RANSAC * ransac = new ImprovedRANSAC(match.pairNum, modelMatchedKeyptPos, patchMatchedKeyptPos,
											 models[match.modelIndex].getNum(), patch.getNum(), models[match.modelIndex].getPositions(), patch.getPositions());
		transformationMatrixs.push_back(ransac->getTransfomationMatrix());

		errors.push_back(computeMatchingError(models[match.modelIndex].getNum(),
											  patch.getNum(),
											  models[match.modelIndex].getPositions(),
											  patch.getPositions()));

		++matchNum;

		delete [] patchMatchedKeyptPos;
		delete [] modelMatchedKeyptPos;
		delete ransac;
	}
}

double MatchingResultParser::computeMatchingError(int keypointNum1, int keypointNum2, const Eigen::Vector3f * keypointPositions1, const Eigen::Vector3f * keypointPositions2)
{
	double matchingError = 0;
	Eigen::Matrix3f rotateMat = transformationMatrixs.back().topLeftCorner(3, 3);
	Eigen::Vector3f transVec = transformationMatrixs.back().topRightCorner(3, 1);

	// Create pcl pointcloud for keypoints 1 & 2
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointCloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointCloud2(new pcl::PointCloud<pcl::PointXYZ>);

	// Copy pointcloud data, transform keypoints 1 by transformation matrix
	keypointCloud1->width = keypointNum1;
	keypointCloud1->height = 1;
	keypointCloud1->points.resize(keypointCloud1->width * keypointCloud1->height);

	keypointCloud2->width = keypointNum2;
	keypointCloud2->height = 1;
	keypointCloud2->points.resize(keypointCloud2->width * keypointCloud2->height);

	for (int i = 0; i < keypointNum1; ++i)
	{
		Eigen::Vector3f transformedPoint = rotateMat * keypointPositions1[i] + transVec;
		keypointCloud1->points[i].x = transformedPoint[0];
		keypointCloud1->points[i].y = transformedPoint[1];
		keypointCloud1->points[i].z = transformedPoint[2];

	}
	for (int i = 0; i < keypointNum2; ++i)
	{
		keypointCloud2->points[i].x = keypointPositions2[i][0];
		keypointCloud2->points[i].y = keypointPositions2[i][1];
		keypointCloud2->points[i].z = keypointPositions2[i][2];
	}

	// Generate kd-tree
	pcl::KdTreeFLANN<pcl::PointXYZ> keypointKDTree1;
	pcl::KdTreeFLANN<pcl::PointXYZ> keypointKDTree2;

	keypointKDTree1.setInputCloud(keypointCloud1);
	keypointKDTree2.setInputCloud(keypointCloud2);

	// Compute matching error from cloud1 to cloud2
	int K = 1;
	int totalNum = 0;
	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointKNNSquaredDistance(K);
	for (int i = 0; i < keypointNum1; ++i)
	{
		pcl::PointXYZ searchPoint;
		searchPoint = keypointCloud1->points[i];
		// Nearest neighbor search
		if (keypointKDTree2.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
		{
			if (pointIdxKNNSearch[0] > paraMinDistance)
			{
				matchingError += pow(pointKNNSquaredDistance[0], 2);
				++totalNum;
			}
		}
	}

	// Compute matching error from cloud2 to cloud1
	for (int i = 0; i < keypointNum2; ++i)
	{
		pcl::PointXYZ searchPoint;
		searchPoint = keypointCloud2->points[i];
		// Nearest neighbor search
		if (keypointKDTree1.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
		{
			if (pointIdxKNNSearch[0] > paraMinDistance)
			{
				matchingError += pow(pointKNNSquaredDistance[0], 2);
				++totalNum;
			}
		}
	}
	//matchingError /= (keypointNum1 * keypointNum2);
	if (totalNum != 0)
		matchingError /= totalNum;
	matchingError = sqrt(matchingError);

	return matchingError;
}
