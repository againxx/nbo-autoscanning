#ifndef MATCHING_RESULT_PARSER_H_
#define MATCHING_RESULT_PARSER_H_

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "RANSAC.h"
#include "ImprovedRANSAC.h"
#include "PartialMatching.h"

struct MatchingResultParser
{
	public:
		MatchingResultParser() = default;
		MatchingResultParser(const std::vector<Match> & matches, const std::vector<KeypointRepresentation> & models, const KeypointRepresentation & patch);
		~MatchingResultParser() = default;

		std::vector<std::string> modelNames;
		std::vector<Eigen::Matrix4f> transformationMatrixs;
		std::vector<double> errors;

		int matchNum = 0;
	private:
		double computeMatchingError(int keypointNum1, int keypointNum2, const Eigen::Vector3f * keypointPositions1, const Eigen::Vector3f * keypointPositions2);

		double paraMinDistance = 0.005;
};

#endif /* ifndef MATCHING_RESULT_PARSER_H_ */
