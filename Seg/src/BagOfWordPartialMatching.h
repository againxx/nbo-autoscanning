#ifndef BAG_OF_WORD_PARTIAL_MATCHING_H_
#define BAG_OF_WORD_PARTIAL_MATCHING_H_

#include <queue>
#include "PartialMatching.h"
#include "PartialCorrespondence.h"

class BagOfWordPartialMatching : public PartialMatching
{
	public:
		BagOfWordPartialMatching() = default;
		BagOfWordPartialMatching(const std::vector<KeypointRepresentation> & models);
		~BagOfWordPartialMatching() = default;

		std::vector<Match> searchPartialMatchedModel(const KeypointRepresentation & patch);

	private:
		void retrieveSimilarModels();
		void computeCorrespondingKeypointPairs();

		std::vector<Match> resultMatches;

		const int paraCandidateCount = 5;
};

#endif /* BAG_OF_WORD_PARTIAL_MATCHING_H_ */
