#ifndef DATASET_GENERATOR_H_
#define DATASET_GENERATOR_H_

#include <string>
#include "DescriptorGenerator.h"
#include "KeypointRepresentation.h"
#include "Utils/DirectoryReader.h"
#include "Utils/FileNameParser.h"

class DatasetGenerator
{
	public:
		DatasetGenerator(int keyptNum = 500, const std::string & keyptFilePostfix = "keypts.txt")
			: keypointNum(keyptNum), keypointFilePostfix(keyptFilePostfix) { }

		virtual ~DatasetGenerator() = default;

		virtual void generate(const std::string & pointCloudDir, const std::string & datasetDir) = 0;

		// SET functions
		void setKeypointNum(int keyptNum) { keypointNum = keyptNum; }
		void setKeypointFilePostfix(const std::string & keyptFilePostfix) { keypointFilePostfix = keyptFilePostfix; }

	protected:
		int keypointNum;
		std::string keypointFilePostfix;
};


#endif /* DATASET_GENERATOR_H_ */
