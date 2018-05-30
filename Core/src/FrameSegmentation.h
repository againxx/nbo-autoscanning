#ifndef FrameSegmentation_H_
#define FrameSegmentation_H_

#include <vector>
#include <map>
#include <deque>
#include <set>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <algorithm>
#include <limits>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>
#include "Utils/Resolution.h"

class FrameSegmentation
{
	public:
		FrameSegmentation() = default;
		~FrameSegmentation();
		
		void process(); 

		void clearMergingPairs();

		void recycleOverridedLabels(std::set<unsigned short> && existingLabels);

		void insertMergingPairs(const std::vector<std::pair<unsigned short, unsigned short>> & pairs);

		bool needMerging() const;

		void removeGroundPlainMergingPairs(const std::vector<unsigned short> & labels);

		// SET functions
		void setFrameData(const unsigned char * rgb,
						  const unsigned short * depth,
						  const float * rawVertexes,
						  const float * rawNormals,
						  const float * projectedVertexes,
						  const float * projectedNormals,
						  const unsigned short * projectedLabels);

		// GET functions
		const unsigned short * getPropagatedLabelMap() const;
        const std::vector<std::pair<unsigned short, unsigned short>> & getMergingPairs() const;
		unsigned short getCurrentLabelRange() const;
		
		static constexpr unsigned int maxMergingPairSize = 100;
		
    private:
        void segmentNewFrame();

        void labelPropagation();

		void updateMergingPairs();

		void compressFrameData();

		int localPooling(int l1, int l2, int l3, int l4);
		cv::Vec3f localPooling(cv::Vec3f v1, cv::Vec3f v2, cv::Vec3f v3, cv::Vec3f v4);

        cv::Mat computeContours(cv::Mat grayMap, int thresholdValue, size_t minSize);

        cv::Mat getLabel(cv::Mat src);

		void fillLabelPool(int num);

		void showColoredLabelMap(const std::string & imageName, cv::Mat labelMap);

		cv::Mat rgbMap;
		cv::Mat depthMap;
		cv::Mat rawVertexMap;
		cv::Mat rawNormalMap;
		cv::Mat projectedVertexMap;
		cv::Mat projectedNormalMap;
		cv::Mat projectedLabelMap;

		cv::Mat currentLabelMap;
		cv::Mat propagatedLabelMap;

		unsigned short newLabel = 1;

		std::deque<unsigned short> labelPool;

		std::vector<std::pair<unsigned short, unsigned short>> mergingPairs;
		std::map<std::pair<unsigned short, unsigned short>, int> mergingPairCandidates;

		// PARAMETERS
		float paraConcavityThreshold = 0.96;
		float paraNoramlThreshold = 20;
		float paraPropagationOverlapThreshold = 0.3;
		float paraMergingOverlapThreshold = 0.2;
		int   paraMinPatchSize = 10;
		int   paraMergingCountThreshold = 5;
};

#endif /* FrameSegmentation_H_ */
