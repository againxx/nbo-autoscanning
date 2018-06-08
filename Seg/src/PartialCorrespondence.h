#ifndef PARTIAL_CORRESPONDENCE_H_
#define PARTIAL_CORRESPONDENCE_H_

#include <iostream>  
#include <limits>
#include <queue>
#include <algorithm>
#include <Eigen/Sparse>  
#include <Eigen/SparseQR>
#include <Eigen/OrderingMethods>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <memory>
#include <time.h>

#include "KeypointRepresentation.h"

class PartialCorrespondence
{
	public:
		PartialCorrespondence() = default;
		PartialCorrespondence(const KeypointRepresentation & firstKeyptRepr, const KeypointRepresentation & secondKeyptRepr);
		~PartialCorrespondence() = default;

		void process();

		// SET functions
		void setFirstKeypointRepresentation(const KeypointRepresentation & firstKeyptRepr) { firstKeypointRepresentation = firstKeyptRepr; clusterFirstKeypointIndexes(); }
		void setSecondKeypointRepresentation(const KeypointRepresentation & secondKeyptRepr) { secondKeypointRepresentation = secondKeyptRepr; }

		// GET functions
		const std::vector<std::pair<int, int>> & getPrunedCorrespondence() const { return prunedCorrespondence; }

	private:
		void initializeRedundantCorrespondence();
		void pruneRedundantCorrespondence();

		void clusterFirstKeypointIndexes();

		KeypointRepresentation firstKeypointRepresentation;		// first for patch
		KeypointRepresentation secondKeypointRepresentation;	// second for model

		std::vector<std::vector<int>> clusteredFirstKeypointIndexes;
		std::vector<std::pair<int, int>> redundantCorrespondence;
		std::vector<std::pair<int, int>> prunedCorrespondence;

		const double paraDescriptorDisThreshold = 5;
		const int paraRedundantNum = 2;
		const double paraConfidence = 0.7;
		const double paraSigma = 0.1;
		const double paraMaxDistance = 0.8;
		const int paraMaxCorrespondenceNum = 25;
};

#endif /* PARTIAL_CORRESPONDENCE_H_ */
