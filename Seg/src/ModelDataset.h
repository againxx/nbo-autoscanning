#ifndef MODEL_DATASET_
#define MODEL_DATASET_

#include <iostream>
#include <vector>
#include <array>
#include <map>
#include <stdexcept>
#include <fstream>
#include <random>
//#include <unistd.h>
#include <Eigen/Dense>

#include "DescriptorGenerator.h"
#include "KeypointRepresentation.h"
#include "Utils/DirectoryReader.h"
#include "Utils/FileNameParser.h"

class ModelDataset
{
	public:
		ModelDataset() = default;
		ModelDataset(const std::string & datasetDir);
		~ModelDataset() = default;

		void load(const std::string & datasetDir);

		void clearAllData();

		void splitInTwo(ModelDataset & subDataset1, ModelDataset & subDataset2);

		// SET functions
		void setKeypointFilePostfix(const std::string & keyptFilePostfix) { keypointFilePostfix = keyptFilePostfix; }
		void setData(const std::vector<KeypointRepresentation> & keyptRepr,
					 const std::vector<std::string> & categories,
					 const std::vector<std::string> & names);

		// GET functions
		const std::string & getModelName(int index) const { return modelNames[index]; }
		const std::string & getModelCategory(int index) const { return modelCategories[index]; }
		const std::string & getKeypointFilePostfix() const { return keypointFilePostfix; }
		int 				getModelNum() const { return modelNum; }
		size_t 				getModelCategoryNum() const { return modelCategories.size(); }
		//int 				getKeypointNum() const { return keypointNum; }
		const std::vector<KeypointRepresentation> & getKeypointRepresentation() const { return keypointRepresentation; }
		std::vector<KeypointRepresentation> & getKeypointRepresentation()
		{
			return const_cast<std::vector<KeypointRepresentation> &>(static_cast<const ModelDataset &>(*this).getKeypointRepresentation());
		}
		std::vector<KeypointRepresentation> getCompleteModelsKeypointRepresentation() const;

	private:
		//void copyLocalTDF(const std::string & sourcePath, const std::string & destPath);

		// Data
		std::vector<KeypointRepresentation> keypointRepresentation;
		std::vector<std::string> modelCategories;
		std::vector<std::string> modelNames;

		std::string keypointFilePostfix = "keypts.txt";

		size_t modelNum = 0;
};


#endif /* ifndef MODEL_DATASET_ */
