#include "RANSAC.h"

RANSAC::RANSAC(int num, const Eigen::Vector3f * matchedKeyptPos1, const Eigen::Vector3f * matchedKeyptPos2)
  : pairNum(num),
	matchedKeypointPositions1(matchedKeyptPos1),
	matchedKeypointPositions2(matchedKeyptPos2)
{

}

Eigen::Matrix4f RANSAC::getTransfomationMatrix() const
{
	constexpr float maxDiff = 0.001;
	constexpr int includedPairNum = 3;

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
	int loopNum = 15 * pairNum;
	int inlinerNum, maxInliner = -1;
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

		inlinerNum = 0;
		for (int j = 0; j < pairNum; ++j)
		{
			float x1 = matchedKeypointPositions1[i][0];
			float y1 = matchedKeypointPositions1[i][1];
			float z1 = matchedKeypointPositions1[i][2];

			float x2 = matchedKeypointPositions2[i][0];
			float y2 = matchedKeypointPositions2[i][1];
			float z2 = matchedKeypointPositions2[i][2];

			float x3 = x1 * transformation2(0, 0) + y1 * transformation2(0, 1) + z1 * transformation2(0, 2) + transformation2(0, 3);
			float y3 = x1 * transformation2(1, 0) + y1 * transformation2(1, 1) + z1 * transformation2(1, 2) + transformation2(1, 3);
			float z3 = x1 * transformation2(2, 0) + y1 * transformation2(2, 1) + z1 * transformation2(2, 2) + transformation2(2, 3);

			float diff = pow(x3 - x2, 2) + pow(y3 - y2, 2) + pow(z3 - z2, 2);
			if (diff < maxDiff)
				inlinerNum++;
		}
		if (maxInliner < inlinerNum)
		{
			maxInliner = inlinerNum;
			transformation1 = transformation2;
		}
	}

	Eigen::Matrix4f transformationMatrix;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			transformationMatrix(i, j) = transformation1(i, j);
		}
	}

	return transformationMatrix;
}
