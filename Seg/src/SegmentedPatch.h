#ifndef SEGMENTED_PATCH_H_
#define SEGMENTED_PATCH_H_

#include <string>
#include <fstream>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include <Eigen/LU>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "Utils/BoundingBox.h"
#include "Utils/PlyReader.h"
#include "Utils/PlyWriter.h"

class SegmentedPatch
{
	public:
		SegmentedPatch() = default;
		SegmentedPatch(int num, Eigen::Vector3f * positions, Eigen::Vector3f * colors, Eigen::Vector3f * normals, float * radii, unsigned short label);
		SegmentedPatch(int num, const Eigen::Vector4f * mapData, unsigned short label, float confidence);
		SegmentedPatch(const std::string & filePath);
		~SegmentedPatch();

		SegmentedPatch & operator+=(const SegmentedPatch & otherPatch);
		SegmentedPatch & operator*=(const Eigen::Matrix4f & transMat);

		void save(const std::string & filePath);
		void load(const std::string & filePath);
		void saveLocalPly(const Eigen::Vector3f & keypoint, const std::string & filePath);

		// SET functions
		void setBoundingBox(const BoundingBox & bd) { boundingBox = bd; }
		void setCenterPosition(const Eigen::Vector3f & cenPos) { centerPosition = cenPos; }
		void setPointData(int num, Eigen::Vector3f * positions, Eigen::Vector3f * colors, Eigen::Vector3f * normals, float * radii);
		void setLabelIndexes(const std::vector<unsigned short> & indexes) { labelIndexes = indexes; }
		void setLabelIndexes(unsigned short index) { labelIndexes.clear(); labelIndexes.push_back(index); }

		// GET functions
		const std::vector<unsigned short> & getLabelIndexes() const { return labelIndexes; } 
		int getPointNum() const {return pointNum;}
		const Eigen::Vector3f * getPointPositions() const { return pointPositions.get(); }
		const Eigen::Vector3f * getPointColors() const { return pointColors.get(); }
		const Eigen::Vector3f * getPointNormals() const { return pointNormals.get(); }
		const float * getPointRadii() const { return pointRadii.get(); }
		Eigen::Vector3f getCenterPosition() const { return centerPosition; }
		const BoundingBox & getBoundingBox() const { return boundingBox; }
		
		// IS functions
		bool hasColor() const { return static_cast<bool>(pointColors); }
		bool hasNormal() const { return static_cast<bool>(pointNormals); }
		bool hasRadius() const { return static_cast<bool>(pointRadii); }

	private:
		void denoise();
		
	private:
		int pointNum = 0;

		std::shared_ptr<Eigen::Vector3f> pointPositions;
		std::shared_ptr<Eigen::Vector3f> pointColors;
		std::shared_ptr<Eigen::Vector3f> pointNormals;
		std::shared_ptr<float>	pointRadii;

		Eigen::Vector3f  centerPosition;

		BoundingBox boundingBox;

		std::vector<unsigned short> labelIndexes;

		static constexpr double paraThresholdClose = 0.08;

		friend SegmentedPatch operator+(const SegmentedPatch & lPatch, const SegmentedPatch & rPatch);
		friend SegmentedPatch operator*(const SegmentedPatch & patch, const Eigen::Matrix4f & transMat);
		friend bool isAdjacent(const SegmentedPatch & lPatch, const SegmentedPatch & rPatch);
};

SegmentedPatch operator+(const SegmentedPatch & lPatch, const SegmentedPatch & rPatch);
SegmentedPatch operator*(const SegmentedPatch & patch, const Eigen::Matrix4f & transMat);
bool isAdjacent(const SegmentedPatch & lPatch, const SegmentedPatch & rPatch);

#endif /* SEGMENTED_PATCH_H_ */
