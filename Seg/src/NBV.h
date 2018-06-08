#ifndef NBV_H_
#define NBV_H_

#include <vector>
#include <array>
#include <queue>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>

#include <pcl/common/io.h>  
#include <pcl/io/io.h>  
#include <pcl/point_cloud.h>  
#include <pcl/io/ply_io.h>  
#include <pcl/console/parse.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "SegmentedPatch.h"
#include "MatchingResultParser.h"
#include "Cuda/cudafuncs.cuh"

class NBV
{
	public:
		NBV() = default;
		NBV(const SegmentedPatch & nboPatch,
			const std::vector<SegmentedPatch> & matchedModels,
			const std::vector<double> & matchingErrors);

		~NBV() = default;

		std::queue<std::array<float, 6>> getNBVQueue(const Eigen::Matrix4f & camPose);

    private:
		void computeConfidence(const std::vector<double> & matchingErrors);

		float computeViewScore(const Eigen::Vector3f & lookAtPosition, const Eigen::Vector3f & viewPosition);

		float * voxelize(const std::vector<SegmentedPatch> & models,
						 const SegmentedPatch & patch,
						 std::array<float, 3> & voxelGridOrigin,
						 std::array<int, 3> & voxelGridDim);

		float getEntropy(int id, std::vector<double> confidence, float* voxelTDF, const std::array<int, 3> & voxel_grid_dim);

		void rayTraceBlocks(float *voxel,
							float *midPt,
							const std::array<float, 3> & voxel_grid_origin,
							const std::array<int, 3> & voxel_grid_dim,
							float *knownVoxel,
							float coe_flag);

		void transformPointsToViewFrame(SegmentedPatch & pointCloud, const Eigen::Vector3f & lookAtPosition, const Eigen::Vector3f & viewPosition);

		Eigen::Matrix4f computeNBVPose(const Eigen::Vector3f & lookAtPosition, const Eigen::Vector3f & viewPosition);

	private:
		SegmentedPatch patch;

		int modelNum;
		std::vector<double> confidence;
		std::vector<SegmentedPatch> models;

		int paraViewNum = 16;
		double paraMoveCostWeight = 500;
		double paraMinViewDistance = 1.0;
		double paraGaussFunctionCenter = 0.0;
		double paraGaussFunctionWidth = 0.01;
		float paraVoxelSize = 0.01;
		int paraVoxelGridPadding = 5;
};


#endif
