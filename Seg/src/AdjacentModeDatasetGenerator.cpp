#include "AdjacentModeDatasetGenerator.h"

void AdjacentModeDatasetGenerator::generate(const std::string & pointCloudDir, const std::string & datasetDir)
{
	loadOriginalPatches(pointCloudDir);

	combineAdjacentPatches();

	saveCombinedPatches(pointCloudDir);

	std::cout << "Successfully generated " << generateKeypointFiles(datasetDir) << " keypoint file(s)." << std::endl; 
}

void AdjacentModeDatasetGenerator::loadOriginalPatches(const std::string & pointCloudDir)
{
	DirectoryReader dirReader(pointCloudDir);
	if (dirReader.isEmpty())
	{
		throw std::runtime_error(pointCloudDir + "\nError: couldn't read point cloud directory!");
	}

	std::string fileName, filePath;
	while (dirReader.getNextFile(fileName, filePath))
	{
		FileNameParser fileNameParser(fileName);

		auto modelName = fileNameParser.getModelName();
		if (find(originalModelNames.cbegin(), originalModelNames.cend(), modelName) != originalModelNames.cend())
			continue;
		originalModelNames.push_back(modelName);

		auto objectName = fileNameParser.getObjectName();

		originalPatches.push_back(SegmentedPatch(filePath));
		originalPatchIndexesInSameObject[objectName].push_back(originalPatches.size() - 1);
	}
}

void AdjacentModeDatasetGenerator::combineAdjacentPatches()
{
	std::vector<int> singlePatchIndexes;
	for (const auto & patchIndexes : originalPatchIndexesInSameObject)
	{
		if (patchIndexes.second.size() == 1)
		{
			singlePatchIndexes.push_back(patchIndexes.second[0]);

			combinedModelNames.push_back(originalModelNames[patchIndexes.second[0]] + "w");
			combinedPatches.push_back(originalPatches[patchIndexes.second[0]]);
			objectData[patchIndexes.first] = combinedPatches.back();
			continue;
		}
		else if (patchIndexes.second.size() == 2)
		{
			int patchIndex1 = patchIndexes.second[0];
			int patchIndex2 = patchIndexes.second[1];

			std::string modelName1 = originalModelNames[patchIndex1];
			std::string modelName2 = originalModelNames[patchIndex2];
			auto foundPlace = modelName1.find_last_of("_");
			std::string patchName1 = modelName1.substr(foundPlace + 1);
			foundPlace = modelName2.find_last_of("_");
			std::string patchName2 = modelName2.substr(foundPlace + 1);

			if (std::stoi(patchName1) > std::stoi(patchName2))
				std::swap(patchName1, patchName2);

			combinedModelNames.push_back(patchIndexes.first + "_" + patchName1 + "+" + patchName2 + "w");
			combinedPatches.push_back(originalPatches[patchIndex1] + originalPatches[patchIndex2]);
			objectData[patchIndexes.first] = combinedPatches.back();
			continue;
		}

		std::cout << "Object: " << patchIndexes.first << " has" << std::endl;

		SegmentedPatch completeObject;
		std::vector<std::string> completeObjectPatchNames;
		std::set<std::pair<std::string, std::string>> adjacentPatchNames;

		for (size_t i = 0; i < patchIndexes.second.size(); ++i)
		{
			int patchIndex1 = patchIndexes.second[i];

			std::string modelName1 = originalModelNames[patchIndex1];
			std::cout << modelName1 << std::endl;
			auto foundPlace = modelName1.find_last_of("_");
			std::string patchName1 = modelName1.substr(foundPlace + 1);

			completeObject += originalPatches[patchIndex1];
			completeObjectPatchNames.push_back(patchName1);

			for (size_t j = i + 1; j < patchIndexes.second.size(); ++j)
			{
				int patchIndex2 = patchIndexes.second[j];
				if (!isAdjacent(originalPatches[patchIndex1], originalPatches[patchIndex2]))
					continue;

				std::string modelName2 = originalModelNames[patchIndex2];
				auto foundPlace = modelName2.find_last_of("_");
				std::string patchName2 = modelName2.substr(foundPlace + 1);

				std::pair<std::string, std::string> adjacentPatchName = std::make_pair(patchName1, patchName2);
				if (std::stoi(adjacentPatchName.first) > std::stoi(adjacentPatchName.second))
					std::swap(adjacentPatchName.first, adjacentPatchName.second);

				if (adjacentPatchNames.count(adjacentPatchName) == 0)
				{
					adjacentPatchNames.insert(adjacentPatchName);
					combinedPatches.push_back(originalPatches[patchIndex1] + originalPatches[patchIndex2]);
					combinedModelNames.push_back(patchIndexes.first + "_" + adjacentPatchName.first + "+" + adjacentPatchName.second);
				}
			}
		}

		combinedPatches.push_back(completeObject);
		std::sort(completeObjectPatchNames.begin(), completeObjectPatchNames.end());

		std::string completeObjectName = patchIndexes.first + "_";
		for (size_t i = 0; i < completeObjectPatchNames.size(); ++i)
		{
			completeObjectName += completeObjectPatchNames[i];
			if (i < completeObjectPatchNames.size() - 1)
				completeObjectName += "+";
		}
		combinedModelNames.push_back(completeObjectName + "w");		// use 'w' to represent whole object
		objectData[patchIndexes.first] = completeObject;
	}
	removeOriginalSinglePatches(singlePatchIndexes);
}

void AdjacentModeDatasetGenerator::saveCombinedPatches(const std::string & saveDir)
{
	std::string saveDirWithSlash = getDirPathWithSlash(saveDir);

	for (size_t i = 0; i < combinedPatches.size(); ++i)
	{
		combinedPatches[i].save(saveDirWithSlash + combinedModelNames[i] + ".ply");
	}
}

int AdjacentModeDatasetGenerator::generateKeypointFiles(const std::string & saveDir)
{
	std::string saveDirWithSlash = getDirPathWithSlash(saveDir);

	int fileNum = 0;
	for (size_t i = 0; i < originalPatches.size(); ++i)
	{
		if (generateOneKeypointFile(originalPatches[i], originalModelNames[i], saveDirWithSlash))
			++fileNum;
	}
	for (size_t i = 0; i < combinedPatches.size(); ++i)
	{
		if (generateOneKeypointFile(combinedPatches[i], combinedModelNames[i], saveDirWithSlash))
			++fileNum;
	}
	return fileNum;
}

bool AdjacentModeDatasetGenerator::generateOneKeypointFile(const SegmentedPatch & patch, const std::string & modelName, const std::string & saveDir)
{
	DescriptorGenerator descriptorGenerator;
	descriptorGenerator.setSamplingMode(DescriptorGenerator::UNIFORM_MODE);

	auto foundPlace = modelName.find_last_of("_");
	auto objectName = modelName.substr(0, foundPlace);

	descriptorGenerator.setPointData(patch, objectData[objectName]);
	KeypointRepresentation keyptRepr = descriptorGenerator.computeDescriptor(keypointNum);

	std::ofstream keypointOutputFile(saveDir + modelName + "." + keypointFilePostfix);
	try
	{
		if (!keypointOutputFile)
		{
			throw std::runtime_error(saveDir + "\nError: save directory is invalid!");
		}
		keypointOutputFile << keyptRepr;
		keypointOutputFile.close();
	}
	catch (std::runtime_error err)
	{
		std::cerr << err.what() << std::endl;
		keypointOutputFile.close();
		return false;
	}
	return true;
}

void AdjacentModeDatasetGenerator::removeOriginalSinglePatches(std::vector<int> singlePatchIndexes)
{
	std::sort(singlePatchIndexes.begin(), singlePatchIndexes.end(), std::greater<int>());

	for (auto indexIter = singlePatchIndexes.cbegin(); indexIter != singlePatchIndexes.cend(); ++indexIter)
	{
		originalPatches.erase(originalPatches.begin() + *indexIter);
		originalModelNames.erase(originalModelNames.begin() + *indexIter);
	}
}
