#include "ImprovedRANSAC.h"

ImprovedRANSAC::ImprovedRANSAC(int num, const Eigen::Vector3f * matchedKeyptPos1, const Eigen::Vector3f * matchedKeyptPos2,
							   int totalKeyptNum1, int totalKeyptNum2, const Eigen::Vector3f * totalKeyptPos1, const Eigen::Vector3f * totalKeyptPos2)
 : RANSAC(num, matchedKeyptPos1, matchedKeyptPos2),
   totalKeypointNum1(totalKeyptNum1),
   totalKeypointNum2(totalKeyptNum2),
   totalKeypointPositions1(totalKeyptPos1),
   totalKeypointPositions2(totalKeyptPos2)
{

}

Eigen::Matrix4f ImprovedRANSAC::getTransfomationMatrix() const
{
	Eigen::Matrix4f transformationMatrix;
	int includedPairNum = 3;

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointCloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointCloud2(new pcl::PointCloud<pcl::PointXYZ>);

	keypointCloud1->width = includedPairNum;
	keypointCloud1->height = 1;
	keypointCloud1->is_dense = false;
	keypointCloud1->points.resize(keypointCloud1->width * keypointCloud1->height);

	keypointCloud2->width = includedPairNum;
	keypointCloud2->height = 1;
	keypointCloud2->is_dense = false;
	keypointCloud2->points.resize(keypointCloud2->width * keypointCloud2->height);

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation1;
	int loopNum = 30 * pairNum;
	double minMatchingError = std::numeric_limits<double>::max();
	double matchingError;
	std::random_device rd;
	std::uniform_int_distribution<int> distribution(0, pairNum - 1);

	for (int i = 0; i < loopNum; ++i)
	{
		int randomIndex[3];
		int temp;
		for (int j = 0; j < includedPairNum; ++j)
		{
			int k;
			do
			{
				temp = distribution(rd);
				for (k = 0; k < j; ++k)
				{
					if (temp == randomIndex[k])
						break;
				}
			}
			while (j != k);
			randomIndex[j] = temp;
		}

		for (size_t j = 0; j < keypointCloud1->points.size(); ++j)
		{
			keypointCloud1->points[j].x = matchedKeypointPositions1[randomIndex[j]][0];
			keypointCloud1->points[j].y = matchedKeypointPositions1[randomIndex[j]][1];
			keypointCloud1->points[j].z = matchedKeypointPositions1[randomIndex[j]][2];
		}

		for (size_t j = 0; j < keypointCloud2->points.size(); ++j)
		{
			keypointCloud2->points[j].x = matchedKeypointPositions2[randomIndex[j]][0];
			keypointCloud2->points[j].y = matchedKeypointPositions2[randomIndex[j]][1];
			keypointCloud2->points[j].z = matchedKeypointPositions2[randomIndex[j]][2];
		}

		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
		TESVD.estimateRigidTransformation(*keypointCloud1, *keypointCloud2, transformation2);

		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				transformationMatrix(i, j) = transformation2(i, j);
			}
		}
		matchingError = computeMatchingError(transformationMatrix);
		if (matchingError < minMatchingError)
		{
			minMatchingError = matchingError;
			transformation1 = transformation2;
		}
	}

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			transformationMatrix(i, j) = transformation1(i, j);
		}
	}

	return transformationMatrix;
}

double ImprovedRANSAC::computeMatchingError(const Eigen::Matrix4f & transformationMatrix) const
{
	double matchingError = 0;
	Eigen::Matrix3f rotateMat = transformationMatrix.topLeftCorner(3, 3);
	Eigen::Vector3f transVec = transformationMatrix.topRightCorner(3, 1);

	// Create pcl pointcloud for keypoints 1 & 2
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointCloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypointCloud2(new pcl::PointCloud<pcl::PointXYZ>);

	// Copy pointcloud data, transform keypoints 1 by transformation matrix
	keypointCloud1->width = totalKeypointNum1;
	keypointCloud1->height = 1;
	keypointCloud1->points.resize(keypointCloud1->width * keypointCloud1->height);

	keypointCloud2->width = totalKeypointNum2;
	keypointCloud2->height = 1;
	keypointCloud2->points.resize(keypointCloud2->width * keypointCloud2->height);

	for (int i = 0; i < totalKeypointNum1; ++i)
	{
		Eigen::Vector3f transformedPoint = rotateMat * totalKeypointPositions1[i] + transVec;
		keypointCloud1->points[i].x = transformedPoint[0];
		keypointCloud1->points[i].y = transformedPoint[1];
		keypointCloud1->points[i].z = transformedPoint[2];

	}
	for (int i = 0; i < totalKeypointNum2; ++i)
	{
		keypointCloud2->points[i].x = totalKeypointPositions2[i][0];
		keypointCloud2->points[i].y = totalKeypointPositions2[i][1];
		keypointCloud2->points[i].z = totalKeypointPositions2[i][2];
	}

	// Generate kd-tree
	pcl::KdTreeFLANN<pcl::PointXYZ> keypointKDTree1;
	pcl::KdTreeFLANN<pcl::PointXYZ> keypointKDTree2;

	keypointKDTree1.setInputCloud(keypointCloud1);
	keypointKDTree2.setInputCloud(keypointCloud2);

	// Compute matching error from cloud1 to cloud2
	int K = 1;
	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointKNNSquaredDistance(K);
	for (int i = 0; i < totalKeypointNum1; ++i)
	{
		pcl::PointXYZ searchPoint;
		searchPoint = keypointCloud1->points[i];
		// Nearest neighbor search
		if (keypointKDTree2.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
		{
			matchingError += pow(pointKNNSquaredDistance[0], 2);
		}
	}

	// Compute matching error from cloud2 to cloud1
	for (int i = 0; i < totalKeypointNum2; ++i)
	{
		pcl::PointXYZ searchPoint;
		searchPoint = keypointCloud2->points[i];
		// Nearest neighbor search
		if (keypointKDTree1.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
		{
			matchingError += pow(pointKNNSquaredDistance[0], 2);
		}
	}
	matchingError /= (totalKeypointNum1 + totalKeypointNum2);
	matchingError = sqrt(matchingError);

	return matchingError;
}
