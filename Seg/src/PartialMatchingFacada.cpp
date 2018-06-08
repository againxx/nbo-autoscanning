#include "PartialMatchingFacada.h"

const int PartialMatchingFacada::BOW_ALGORITHM = 0;
const int PartialMatchingFacada::PRIORITY_ALGORITHM = 1;

PartialMatchingFacada::PartialMatchingFacada(const std::string & datasetDir)
 : modelDataset(datasetDir),
   visualWord(paraVisualWordNum)
{
	visualWord.kmeansCluster(modelDataset.getKeypointRepresentation());
	descriptorGenerator.setSamplingMode(DescriptorGenerator::UNIFORM_MODE);
}

PartialMatchingFacada::PartialMatchingFacada(const std::string & datasetDir, const std::string & visualWordFilePath)
 : modelDataset(datasetDir),
   visualWord(paraVisualWordNum)
{
	/* TODO: save cluster infomation in keypoint file <02-10-17, xixia> */
	visualWord.load(visualWordFilePath);

	for (int i = 0; i < modelDataset.getModelNum(); ++i)
	{
		visualWord.assignKeypointsToClusters(modelDataset.getKeypointRepresentation()[i]);
	}

	descriptorGenerator.setSamplingMode(DescriptorGenerator::UNIFORM_MODE);
}

PartialMatchingFacada::~PartialMatchingFacada()
{
	if (partialMatching)
		delete partialMatching;
}

MatchingResultParser PartialMatchingFacada::getMatchingResult(const SegmentedPatch & patch, bool onlyConsiderCompleteModel)
{
	//auto startTime = std::chrono::system_clock::now();
	descriptorGenerator.setPointData(patch);
	return computeMatchingResult(onlyConsiderCompleteModel);
	//auto endTime = std::chrono::system_clock::now();
	//auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	//std::cout << "Spend " << (double)duration.count() * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den  << " seconds matching one patch." << std::endl;
}

MatchingResultParser PartialMatchingFacada::getMatchingResult(const SegmentedPatch & patch, const SegmentedPatch & adjacentPatch, bool onlyConsiderCompleteModel)
{
	//auto startTime = std::chrono::system_clock::now();
	descriptorGenerator.setPointData(patch, adjacentPatch);
	return computeMatchingResult(onlyConsiderCompleteModel);
	//auto endTime = std::chrono::system_clock::now();
	//auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	//std::cout << "Spend " << (double)duration.count() * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den  << " seconds matching one patch." << std::endl;
}

MatchingResultParser PartialMatchingFacada::computeMatchingResult(bool onlyConsiderCompleteModel)
{
	if (partialMatching)
		delete partialMatching;

	if (modelDataset.getCompleteModelsKeypointRepresentation().empty())
		onlyConsiderCompleteModel = false;

    if (partialMatchingAlgorithm == BOW_ALGORITHM)
	    partialMatching = onlyConsiderCompleteModel ? new BagOfWordPartialMatching(modelDataset.getCompleteModelsKeypointRepresentation())
												    : new BagOfWordPartialMatching(modelDataset.getKeypointRepresentation());
    else
        partialMatching = onlyConsiderCompleteModel ? new PriorityQueuePartialMatching(modelDataset.getCompleteModelsKeypointRepresentation())
                                                    : new PriorityQueuePartialMatching(modelDataset.getKeypointRepresentation());

	auto keyptRepr = descriptorGenerator.computeDescriptor(paraKeypointNum);

	visualWord.assignKeypointsToClusters(keyptRepr);

	std::vector<Match> pmResult = partialMatching->searchPartialMatchedModel(keyptRepr);

	return MatchingResultParser(pmResult,
								onlyConsiderCompleteModel ? modelDataset.getCompleteModelsKeypointRepresentation() : modelDataset.getKeypointRepresentation(),
								keyptRepr);
}

void PartialMatchingFacada::setPartialMatchingAlgorithm(int algorithm)
{
	if (algorithm != BOW_ALGORITHM && algorithm != PRIORITY_ALGORITHM)
	{
		std::cerr << "Error: invalid algorithm!" << std::endl;
		return;
	}
	else
	{
		partialMatchingAlgorithm = algorithm;
	}
}
