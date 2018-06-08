#ifndef PRIORITY_QUEUE_PARTIAL_MATCHING_H_
#define PRIORITY_QUEUE_PARTIAL_MATCHING_H_

#include <iostream>
#include <queue>
#include <utility>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <memory>
#include <omp.h>

#include "PartialMatching.h"

class PriorityQueuePartialMatching : public PartialMatching
{
	public:
		PriorityQueuePartialMatching() = default;
		PriorityQueuePartialMatching(const std::vector<KeypointRepresentation> & models);
		~PriorityQueuePartialMatching();

		std::vector<Match> searchPartialMatchedModel(const KeypointRepresentation & patch);
		
	private:
		void priorityDrivenSearch();
		void computeCost(Match & m);
		void removeMatchesFromPriorityQueue(size_t target);
		Match extendMatch(const Match & match, const Match & pair);
		void showResult();

		std::priority_queue<Match, std::vector<Match>, MatchComparison> matchPriorityQueue;
		Match * bestPartialMatch;
		std::vector<std::vector<Match>> correspondPairs;
		std::vector<Match> resultMatches;

		double paraOnePairThreshold = 0.5;
		double paraMultiPairThreshold = 0.3;
		int    paraCandidateCount = 3;
		int    paraCorrespondCount = 5;
		double paraMaxRank = 0.13;
		double paraMaxRadius = 5;
		double paraMaxNormal = 5;
		double paraMaxLength = 0.2;
		double paraMaxOrient = 0.2;
		double paraCoefRank = 20;
		double paraCoefRadius = 0;
		double paraCoefNormal = 0;
		double paraCoefLength = 10;
		double paraCoefOrient = 10;
		double paraExpRank = 2;
		double paraExpRadius = 2;
		double paraExpNormal = 2;
		double paraExpLength = 2;
		double paraExpOrient = 2;
		float  paraMinLength = 0.2;
		int    paraMaxAttemptCount = 1000;
};

#endif 	/* PRIORITY_QUEUE_PARTIAL_MATCHING_H_ */
