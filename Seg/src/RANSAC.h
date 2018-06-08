#ifndef RANSAC_H_
#define RANSAC_H_

#include <memory>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include "ModelDataset.h"

class RANSAC
{
	public:
		RANSAC(int num, const Eigen::Vector3f * matchedKeyptPos1, const Eigen::Vector3f * matchedKeyptPos2);
		virtual ~RANSAC() = default;

		virtual Eigen::Matrix4f getTransfomationMatrix() const;

	protected:
		int pairNum;
		const Eigen::Vector3f * matchedKeypointPositions1;
		const Eigen::Vector3f * matchedKeypointPositions2;
};

#endif /* ifndef RANSAC_H_ */
