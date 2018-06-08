#include "ModelDataset.h"

ModelDataset::ModelDataset(const std::string & datasetDir)
{
	load(datasetDir);
}

void ModelDataset::load(const std::string & datasetDir)
{
	// Clear old data
	clearAllData();

	DirectoryReader dirReader(datasetDir);
	if (dirReader.isEmpty())
	{
		std::cerr << "Error: couldn't read dataset directory!" << std::endl;
		return;
	}

	std::string fileName, filePath;

	// Load model's keypoints files in dir
	while (dirReader.getNextFile(fileName, filePath))
	{
		FileNameParser fileNameParser(fileName);

		// Ignore the wrong file postfix
		if (fileNameParser.getPostfix() != keypointFilePostfix)
			continue;

		// Ignore the same named model
		auto modelName = fileNameParser.getModelName();
		if (find(modelNames.cbegin(), modelNames.cend(), modelName) != modelNames.cend())
			continue;

		KeypointRepresentation keyptRepr;
		keyptRepr.load(filePath);
		
		if (!keyptRepr.isEmpty())
		{
			keypointRepresentation.push_back(keyptRepr);
			++modelNum;

			// Find out model's category by its name
			auto categoryName = fileNameParser.getCategoryName();
			if (find(modelCategories.cbegin(), modelCategories.cend(), categoryName) == modelCategories.cend())
				modelCategories.push_back(categoryName);

			modelNames.push_back(modelName);
			keypointRepresentation.back().setName(modelName);
		}
	}
	std::cout << "Successfully loaded " << modelNum << " models!" << std::endl;
}

void ModelDataset::splitInTwo(ModelDataset & subDataset1, ModelDataset & subDataset2)
{
	subDataset1.clearAllData();
	subDataset2.clearAllData();

	std::vector<KeypointRepresentation> keyptRepr1; 
	std::vector<KeypointRepresentation> keyptRepr2; 
	/* TODO: Try to split neighborhood <30-11-17, xiaxi> */
	for (const auto & originalKeyptRepr : keypointRepresentation)
	{
		std::shared_ptr<Eigen::Vector3f> positions1(new Eigen::Vector3f[originalKeyptRepr.getNum() / 2], [](Eigen::Vector3f * p){delete [] p;});
		std::shared_ptr<Eigen::Vector3f> positions2(new Eigen::Vector3f[originalKeyptRepr.getNum() / 2], [](Eigen::Vector3f * p){delete [] p;});

		std::shared_ptr<Eigen::Vector3f> normals1(new Eigen::Vector3f[originalKeyptRepr.getNum() / 2], [](Eigen::Vector3f * p){delete [] p;});
		std::shared_ptr<Eigen::Vector3f> normals2(new Eigen::Vector3f[originalKeyptRepr.getNum() / 2], [](Eigen::Vector3f * p){delete [] p;});

		std::shared_ptr<float> descriptors1(new float[originalKeyptRepr.getNum() / 2 * KeypointRepresentation::descriptorSize], [](float * p){delete [] p;});
		std::shared_ptr<float> descriptors2(new float[originalKeyptRepr.getNum() / 2 * KeypointRepresentation::descriptorSize], [](float * p){delete [] p;});
		
		std::random_device rd;
		std::uniform_int_distribution<int> distribution(0, originalKeyptRepr.getNum() - 1);
		std::set<unsigned int> keypointIndexes1;
		std::set<unsigned int> keypointIndexes2;

		while (keypointIndexes1.size() < originalKeyptRepr.getNum() / 2)
		{
			int randomIndex;
			do
			{
				randomIndex = distribution(rd);
			}
			while (keypointIndexes1.count(randomIndex) != 0);
			keypointIndexes1.insert(randomIndex);
		}

		for (unsigned int i = 0; i < originalKeyptRepr.getNum(); ++i)
		{
			if (keypointIndexes1.count(i) == 0)
				keypointIndexes2.insert(i);
		}

		int setIndex = 0;
		for (auto indexIter = keypointIndexes1.cbegin(); indexIter != keypointIndexes1.cend(); ++indexIter)
		{
			positions1.get()[setIndex] = originalKeyptRepr.getPositions()[*indexIter];
			normals1.get()[setIndex] = originalKeyptRepr.getNormals()[*indexIter];

			for (int j = 0; j < KeypointRepresentation::descriptorSize; ++j)
				descriptors1.get()[setIndex * KeypointRepresentation::descriptorSize + j] = originalKeyptRepr.getDescriptors()[*indexIter * KeypointRepresentation::descriptorSize + j];

			++setIndex;
		}

		setIndex = 0;
		for (auto indexIter = keypointIndexes2.cbegin(); indexIter != keypointIndexes2.cend(); ++indexIter)
		{
			positions2.get()[setIndex] = originalKeyptRepr.getPositions()[*indexIter];
			normals2.get()[setIndex] = originalKeyptRepr.getNormals()[*indexIter];

			for (int j = 0; j < KeypointRepresentation::descriptorSize; ++j)
				descriptors2.get()[setIndex * KeypointRepresentation::descriptorSize + j] = originalKeyptRepr.getDescriptors()[*indexIter * KeypointRepresentation::descriptorSize + j];

			++setIndex;
		}

		keyptRepr1.emplace_back(originalKeyptRepr.getNum() / 2,
								positions1,
								normals1,
								descriptors1,
								std::shared_ptr<std::set<std::pair<int, int>>>(new std::set<std::pair<int, int>>),
								originalKeyptRepr.getCenterPosition(),
								originalKeyptRepr.getDiameter(),
								originalKeyptRepr.getName());

		keyptRepr2.emplace_back(originalKeyptRepr.getNum() / 2,
								positions2,
								normals2,
								descriptors2,
								std::shared_ptr<std::set<std::pair<int, int>>>(new std::set<std::pair<int, int>>),
								originalKeyptRepr.getCenterPosition(),
								originalKeyptRepr.getDiameter(),
								originalKeyptRepr.getName());
	}
	subDataset1.setData(keyptRepr1, modelCategories, modelNames);
	subDataset2.setData(keyptRepr2, modelCategories, modelNames);
}

void ModelDataset::clearAllData()
{
	keypointRepresentation.clear();
	modelCategories.clear();
	modelNames.clear();
	modelNum = 0;
}

//void ModelDataset::copyLocalTDF(const std::string & sourcePath, const std::string & destPath)
//{
//	std::ifstream inputFile(sourcePath.c_str(), std::ios::binary);
//	std::ofstream outputFile(destPath.c_str(), std::ios::binary);
//
//	outputFile << inputFile.rdbuf();
//
//	inputFile.close();
//	outputFile.close();
//}

std::vector<KeypointRepresentation> ModelDataset::getCompleteModelsKeypointRepresentation() const
{
	std::vector<KeypointRepresentation> completeModelsKeyptRepr;
	for (size_t i = 0; i < modelNames.size(); ++i)
	{
		if (modelNames[i].back() == 'w')
		{
			completeModelsKeyptRepr.push_back(keypointRepresentation[i]);
		}	
	}

	return completeModelsKeyptRepr;
}

void ModelDataset::setData(const std::vector<KeypointRepresentation> & keyptRepr,
						   const std::vector<std::string> & categories,
						   const std::vector<std::string> & names)
{ 
	if (keyptRepr.size() != names.size())
	{
		std::cerr << "Error: data dimensions are inconsistent!" << std::endl;
		return;
	}

	modelNum = keyptRepr.size();
	keypointRepresentation = keyptRepr;
	modelCategories = categories;
	modelNames = names;
}
