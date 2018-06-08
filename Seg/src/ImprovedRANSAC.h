#ifndef IMPROVED_RANSAC_H_
#define IMPROVED_RANSAC_H_

#include "RANSAC.h"

class ImprovedRANSAC : public RANSAC
{
	public:
		ImprovedRANSAC(int num, const Eigen::Vector3f * matchedKeyptPos1, const Eigen::Vector3f * matchedKeyptPos2,
					   int totalKeyptNum1, int totalKeyptNum2, const Eigen::Vector3f * totalKeyptPos1, const Eigen::Vector3f * totalKeyptPos2);
		~ImprovedRANSAC() = default;

		Eigen::Matrix4f getTransfomationMatrix() const;

	private:
		double computeMatchingError(const Eigen::Matrix4f & transformationMatrix) const;

		int totalKeypointNum1;
		int totalKeypointNum2;
		const Eigen::Vector3f * totalKeypointPositions1;
		const Eigen::Vector3f * totalKeypointPositions2;
};

#endif /* ifndef IMPROVED_RANSAC_H_ */
