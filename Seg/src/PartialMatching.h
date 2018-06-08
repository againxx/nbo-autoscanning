#ifndef PARTIAL_MATCHING_H_
#define PARTIAL_MATCHING_H_

#include <vector>
#include <limits>
#include "KeypointRepresentation.h"

struct Match
{
	int pairNum = 0;
	double costValue = std::numeric_limits<double>::max();
	int costTermNum = 0;

	size_t modelIndex = -1;
	std::vector<unsigned int> modelKeypointIndexes;
	std::vector<unsigned int> patchKeypointIndexes;
};

class MatchComparison
{
	public:
		bool operator() (const Match & lMatch, const Match & rMatch)
		{
			return lMatch.costValue > rMatch.costValue;
		}
};

class PartialMatching
{
	public:
		PartialMatching() = default;
		PartialMatching(const std::vector<KeypointRepresentation> & models) : modelData(models) {}
		virtual ~PartialMatching() = default;

		virtual std::vector<Match> searchPartialMatchedModel(const KeypointRepresentation & patch) = 0;
		
	protected:
		std::vector<KeypointRepresentation> modelData;
		KeypointRepresentation patchData;
};

#endif 	/* PARTIAL_MATCHING_H_ */
