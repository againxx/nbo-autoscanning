#ifndef SINGLE_MODE_DATASET_GENERATOR_H_
#define SINGLE_MODE_DATASET_GENERATOR_H_

#include "DatasetGenerator.h"

class SingleModeDatasetGenerator : public DatasetGenerator
{
	public:
		SingleModeDatasetGenerator(int keyptNum = 500, const std::string & keyptFilePostfix = "keypts.txt")
			: DatasetGenerator(keyptNum, keyptFilePostfix) { }

		~SingleModeDatasetGenerator() = default;

		void generate(const std::string & pointCloudDir, const std::string & datasetDir);

};

#endif /* SINGLE_MODE_DATASET_GENERATOR_H_ */
