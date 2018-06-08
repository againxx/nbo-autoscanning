#include "BagOfWordPartialMatching.h"

BagOfWordPartialMatching::BagOfWordPartialMatching(const std::vector<KeypointRepresentation> & models)
 : PartialMatching(models)
{

}

std::vector<Match> BagOfWordPartialMatching::searchPartialMatchedModel(const KeypointRepresentation & patch)
{
	patchData = patch;

	retrieveSimilarModels();
	computeCorrespondingKeypointPairs();

	return resultMatches;
}

void BagOfWordPartialMatching::retrieveSimilarModels()
{
	resultMatches.clear();
	std::priority_queue<Match, std::vector<Match>, MatchComparison> matchPriorityQueue;

	for (size_t i = 0; i < modelData.size(); ++i)
	{
		Match tempMatch;
		tempMatch.modelIndex = i;
		tempMatch.costValue = patchData.computeHistogramDistance(modelData[i]);

		matchPriorityQueue.push(tempMatch);
	}

	for (int i = 0; i < paraCandidateCount; ++i)
	{
		if (matchPriorityQueue.empty())
			break;

		resultMatches.push_back(matchPriorityQueue.top());
		matchPriorityQueue.pop();
	}
}

void BagOfWordPartialMatching::computeCorrespondingKeypointPairs()
{
	PartialCorrespondence partialCorrespondence;
	partialCorrespondence.setFirstKeypointRepresentation(patchData);

	for (auto matchIter = resultMatches.begin(); matchIter != resultMatches.end(); )
	{
		partialCorrespondence.setSecondKeypointRepresentation(modelData[matchIter->modelIndex]);
		partialCorrespondence.process();

		if (partialCorrespondence.getPrunedCorrespondence().size() >= 3)
		{
			matchIter->patchKeypointIndexes.clear();
			matchIter->modelKeypointIndexes.clear();
			matchIter->pairNum = partialCorrespondence.getPrunedCorrespondence().size();
		
			for (const auto & corrPair : partialCorrespondence.getPrunedCorrespondence())
			{
				matchIter->patchKeypointIndexes.push_back(corrPair.first);
				matchIter->modelKeypointIndexes.push_back(corrPair.second);
			}
			++matchIter;
		}
		else
			matchIter = resultMatches.erase(matchIter);

	}
}
