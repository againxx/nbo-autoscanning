#ifndef ADJACENT_MODE_DATASET_GENERATOR_
#define ADJACENT_MODE_DATASET_GENERATOR_

#include <stdexcept>
#include "DatasetGenerator.h"

class AdjacentModeDatasetGenerator : public DatasetGenerator
{
	public:
		AdjacentModeDatasetGenerator(int keyptNum = 500, const std::string & keyptFilePostfix = "keypts.txt")
			: DatasetGenerator(keyptNum, keyptFilePostfix) { }

		~AdjacentModeDatasetGenerator() = default;

		void generate(const std::string & pointCloudDir, const std::string & datasetDir);

	private:
		void loadOriginalPatches(const std::string & pointCloudDir);
		void combineAdjacentPatches();
		void saveCombinedPatches(const std::string & saveDir);
		int generateKeypointFiles(const std::string & saveDir);
		bool generateOneKeypointFile(const SegmentedPatch & patch, const std::string & modelName, const std::string & saveDir);
		void removeOriginalSinglePatches(std::vector<int> singlePatchIndexes);

		std::string getDirPathWithSlash(const std::string & dirPath) const
		{
			if (dirPath.back() != '/')
				return dirPath + '/';
			else
				return dirPath;
		}

		std::vector<SegmentedPatch> originalPatches;
		std::vector<std::string> originalModelNames;
		std::map<std::string, std::vector<int>> originalPatchIndexesInSameObject;

		std::vector<SegmentedPatch> combinedPatches;
		std::vector<std::string> combinedModelNames;

		std::map<std::string, SegmentedPatch> objectData;

};

#endif /* ADJACENT_MODE_DATASET_GENERATOR_ */
