#ifndef DESCRIPTOR_GENERATOR_H_
#define DESCRIPTOR_GENERATOR_H_

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <algorithm>
#include <limits>
#include <chrono>
#include <set>
#include <sys/stat.h>
#include <pcl/point_cloud.h>  
#include <pcl/io/ply_io.h>  
#include "SegmentedPatch.h"
#include "KeypointRepresentation.h"
#include "Cuda/cudafuncs.cuh"

class DescriptorGenerator
{
	public:
		DescriptorGenerator() = default;
		DescriptorGenerator(const SegmentedPatch & patch);
		DescriptorGenerator(const std::string & filePath);
		~DescriptorGenerator();

		KeypointRepresentation computeDescriptor(int keyptNum = 500);

		void saveTDF(const std::string & prefixFileName);
		void saveKeypointLocalTDFs(const std::string & prefixFileName);
		void loadKeypoints(const std::string prefixFileName);

		// SET functions
		void setPointData(const std::string & filePath);
		void setPointData(const SegmentedPatch & patch);
		void setPointData(const SegmentedPatch & patch, const SegmentedPatch & adjacentPatch);
		void setSamplingMode(int mode);

		// GET functions
		int getSamplingMode() const { return samplingMode; }

		static const int RANDOM_MODE, UNIFORM_MODE;

	private:
		void computeTDF();
		void randomlySampleKeypoints();
		void uniformlySampleKeypoints();

		int 	keypointNum = 500;
		float 	voxelSize = 0.01;
		float 	truncMargin = voxelSize * 5;
		int 	voxelGridPadding = 15; 	//in voxels
		int 	batchSize = 50;
		const int 	descriptorSize = 512;

		float 	voxelGridOriginX, voxelGridOriginY, voxelGridOriginZ;
		float 	voxelGridMaxX, voxelGridMaxY, voxelGridMaxZ;
		int 	voxelGridDimX, voxelGridDimY, voxelGridDimZ;

		int 	pointNum = 0;
		float * pointPositions = nullptr;
		float * pointNormals = nullptr;
		float * voxelGridTDF = nullptr;
		std::shared_ptr<Eigen::Vector3f> keypointPositions;
		std::shared_ptr<Eigen::Vector3f> keypointNormals;
		std::shared_ptr<float> keypointDescriptors;
		std::shared_ptr<std::set<std::pair<int, int>>> keypointNeighborhoods;
		float * keypointGrid = nullptr;
		BoundingBox boundingBox;
        Eigen::Vector3f centerPosition;
		float 	diameter;

		int adjacentPointNum = 0;
		float * adjacentPointPositions = nullptr;
		float * adjacentPointNormals = nullptr;
		BoundingBox adjacentBoundingBox;

		bool isConsiderAdjacent = false;
		int samplingMode = RANDOM_MODE;
};

#endif	/* DESCRIPTOR_GENERATOR_H_ */
