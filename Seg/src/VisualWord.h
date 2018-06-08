#ifndef VISUAL_WORD_H_
#define VISUAL_WORD_H_

#include <vector>
#include <list>
#include <array>
#include <string>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include "KeypointRepresentation.h"

class VisualWord
{
	public:
		VisualWord(size_t num);
		~VisualWord();

		void kmeansCluster(std::vector<KeypointRepresentation> & keyptReprs);

		void assignKeypointsToClusters(KeypointRepresentation & keyptRepr) const;

		void load(const std::string & filePath);
		void save(const std::string & filePath);

		static const int HARD_ASSIGNMENT, SOFT_ASSIGNMENT;

		// SET functions
		void setClusterNum(size_t num) { clusterNum = num; }
		
		// GET functions
		size_t getClusterNum() const { return clusterNum; }

	private:
		std::vector<int> findCloseClusterCenter(const float * descriptor, size_t num) const;

		size_t clusterNum; 
		std::vector<std::array<float, KeypointRepresentation::descriptorSize>> clusterCenters;
		int clusterAssignmentMode = SOFT_ASSIGNMENT;

		friend std::istream & operator>>(std::istream & is, VisualWord & visualWord);
		friend std::ostream & operator<<(std::ostream & os, const VisualWord & visualWord);
};

std::istream & operator>>(std::istream & is, VisualWord & visualWord);
std::ostream & operator<<(std::ostream & os, const VisualWord & visualWord);

#endif /* VISUAL_WORD_H_ */
