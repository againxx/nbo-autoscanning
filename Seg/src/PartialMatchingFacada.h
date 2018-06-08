#ifndef PARTIAL_MATCHING_FACADA_H_
#define PARTIAL_MATCHING_FACADA_H_

#include "VisualWord.h"
#include "PriorityQueuePartialMatching.h"
#include "BagOfWordPartialMatching.h"
#include "DescriptorGenerator.h"
#include "MatchingResultParser.h"
#include <chrono>

class PartialMatchingFacada
{
	public:
		PartialMatchingFacada(const std::string & datasetDir);
		PartialMatchingFacada(const std::string & datasetDir, const std::string & visualWordFilePath);
		~PartialMatchingFacada();

        // GET functions
		MatchingResultParser getMatchingResult(const SegmentedPatch & patch, bool onlyConsiderCompleteModel = false);
		MatchingResultParser getMatchingResult(const SegmentedPatch & patch, const SegmentedPatch & adjacentPatch, bool onlyConsiderCompleteModel = false);
        int getPartialMatchingAlgorithm() const { return partialMatchingAlgorithm; }

        // SET functions
        void setPartialMatchingAlgorithm(int algorithm);

		static const int BOW_ALGORITHM, PRIORITY_ALGORITHM;

	private:
		MatchingResultParser computeMatchingResult(bool onlyConsiderCompleteModel);

		int paraVisualWordNum = 100;
		int paraKeypointNum = 500;

		ModelDataset modelDataset;
		VisualWord visualWord;

        int partialMatchingAlgorithm = BOW_ALGORITHM;

		DescriptorGenerator descriptorGenerator;
		PartialMatching * partialMatching = nullptr;
};

#endif /* PARTIAL_MATCHING_FACADA_H_ */
