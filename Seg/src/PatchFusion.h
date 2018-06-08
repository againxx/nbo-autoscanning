#ifndef PATCH_FUSION_H_
#define PATCH_FUSION_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <random>
#include <memory>
#include <list>
#include <vector>
#include <chrono>
#include <map>
#include <dirent.h>
#include <sys/stat.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>  

#include "GCO/graph.h"
#include "GCO/GCoptimization.h"
#include "SegmentedPatch.h"
#include "PartialMatchingFacada.h"

class PatchFusion
{
	public:
		PatchFusion(const std::vector<SegmentedPatch> & patches, std::shared_ptr<PartialMatchingFacada> pmFacada);
		PatchFusion(std::shared_ptr<PartialMatchingFacada> pmFacada);
		~PatchFusion() = default;

		void process(const std::vector<unsigned short> & recognizedPatchesByYolo, double probability);
		void testLoop(const std::string & filePath);

		// SET functions
		void setPatchData(const std::vector<SegmentedPatch> & patches);

		// GET functions
		int     								  getPatchNum() const { return patchNum; }
		const std::vector<int> & 				  getGraphCutResult() const { return multiGraphCutResult; }
		const std::vector<SegmentedPatch> & 	  getObjectHypotheses() const { return objectHypotheses; }
		const std::vector<MatchingResultParser> & getObjectHypothesesMatchingResults() const { return objectHypothesesMatchingResults; }
	private:
		void 	preprocess();

		SegmentedPatch findAdjacentPatches(std::vector<int> patchIds);

		void 	retrieveSinglePatches();				// For every raw input patch, do partial matching
		void 	retrieveCoupledPatches();				// For any two adjacent patches, combine them and do partial matching

		void 	computeDataTerms();
		void 	computeSmoothTerms(const std::vector<unsigned short> & recognizedPatchesByYolo, double probability);

		void 	multiGraphCut();

		void 	generateObjectHypotheses();

		inline double gaussFunction(double x);

		void 	loadPatchData(const std::string & patchDir);
		void 	showDataSmoothTerm();
		void 	setDataSmoothTerm(const std::string & filePath);
		void 	saveDataSmoothTerm(const std::string & filePath);

		void 	clearData();
	private:
		std::vector<SegmentedPatch> patchData;
		std::vector<SegmentedPatch> objectHypotheses;
		std::vector<MatchingResultParser> objectHypothesesMatchingResults;

		std::vector<std::vector<bool>> patchConnectFlags;
		std::vector<std::pair<int,int>> patchConnection;

		std::vector<std::vector<double>> dataTerms;
		std::vector<std::vector<double>> smoothTerms;
		//std::vector<std::vector<int>> multiGraphCutResult;
		std::vector<int> multiGraphCutResult;

		std::shared_ptr<PartialMatchingFacada> partialMatchingFacada;

		std::map<std::vector<unsigned short>, MatchingResultParser> matchingResultCache;

		std::vector<std::string> matchedModelCategories;		// with 'null' in the first element

		int 	patchNum;
		const double paraThresholdClose = 0.05;
		const double paraGaussFunctionCenter = 0.002;		// 0.01
		const double paraGaussFunctionWidth = 0.05;		// 0.05 0.02
};

#endif		/* PATCH_FUSION_H_ */
