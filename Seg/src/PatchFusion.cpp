#include "PatchFusion.h"

PatchFusion::PatchFusion(const std::vector<SegmentedPatch> & patches, std::shared_ptr<PartialMatchingFacada> pmFacada)
 : patchData(patches),
   partialMatchingFacada(pmFacada),
   patchNum(patchData.size())
{
	preprocess();
}

PatchFusion::PatchFusion(std::shared_ptr<PartialMatchingFacada> pmFacada)
 : partialMatchingFacada(pmFacada)
{

}

void PatchFusion::preprocess()
{
	// Compute connection between two patches
	patchConnectFlags.resize(patchNum);
	smoothTerms.resize(patchNum);

	for(int i = 0; i < patchNum; ++i)
	{
		patchConnectFlags[i].resize(patchNum, false);
		smoothTerms[i].resize(patchNum, -1);
	}

	for(int i = 0; i < patchNum; ++i)
	{
		for(int j = i + 1; j < patchNum; ++j)
		{
			if (isAdjacent(patchData[i], patchData[j]))
			{
				patchConnection.push_back({i, j});
				patchConnection.push_back({j, i});

				patchConnectFlags[i][j] = patchConnectFlags[j][i] = true;
			}
		}
	}
}

void PatchFusion::process(const std::vector<unsigned short> & recognizedPatchesByYolo, double probability)
{
	computeDataTerms();

	computeSmoothTerms(recognizedPatchesByYolo, probability);

	showDataSmoothTerm();
	//saveDataSmoothTerm("./dataSmooth.txt");
	multiGraphCut();

	generateObjectHypotheses();
}

void PatchFusion::setPatchData(const std::vector<SegmentedPatch> & patches)
{
	clearData();
	patchData = patches;
	patchNum = patchData.size();
	preprocess();
}

void PatchFusion::loadPatchData(const std::string & patchDir)
{
	patchData.clear();
	if (patchDir.empty())
	{
		std::cerr << "Error: patch dir is null." << std::endl;
		return;
	}

	// Open a dir with patch files
	DIR * pDir = opendir(patchDir.c_str());
	if (pDir == nullptr)
	{
		std::cerr << "Error: patch dir is not a valid directory." << std::endl;
		return;
	}

	struct stat fileStat;
	struct dirent * pEntry;
	std::string filePath;
	std::string fileName;

	// Load patch file in dir
	while ((pEntry = readdir(pDir)))
	{
		fileName = pEntry->d_name;
		filePath = patchDir + "/" + fileName;

		// If the file is a directory (or in some way invalid) we'll skip it
		if (stat(filePath.c_str(), &fileStat))
			continue;
		if (S_ISDIR(fileStat.st_mode))
			continue;
		auto found = fileName.find_first_of(".");
		fileName = fileName.substr(0, found);
		SegmentedPatch tempPatch;
		std::cout << filePath << std::endl;
		tempPatch.load(filePath);
		tempPatch.setLabelIndexes(stoi(fileName));
		patchData.push_back(tempPatch);
	}
	closedir(pDir);
	patchNum = patchData.size();
	std::sort(patchData.begin(), patchData.end(), [](const SegmentedPatch & p1, const SegmentedPatch & p2){return (p1.getLabelIndexes()[0] < p2.getLabelIndexes()[0]);});
	std::cout << "Successfully loaded " << patchNum << " patches." << std::endl;
}

void PatchFusion::testLoop(const std::string & filePath)
{
	setDataSmoothTerm(filePath);
	showDataSmoothTerm();
	multiGraphCut();
}

SegmentedPatch PatchFusion::findAdjacentPatches(std::vector<int> patchIds)
{
	SegmentedPatch resultPatch;
	for (auto pId : patchIds)
	{
		resultPatch += patchData[pId];
	}

	for (size_t i = 0; i < patchIds.size(); ++i)
	{
		for (size_t j = 0; j < patchConnectFlags[patchIds[i]].size(); ++j)
		{
			if (patchConnectFlags[patchIds[i]][j])
			{
				bool alreadyHave = false;
				for (size_t k = 0; k < resultPatch.getLabelIndexes().size(); ++k)
				{
					if (patchData[j].getLabelIndexes()[0] == resultPatch.getLabelIndexes()[k])
					{
						alreadyHave = true;
						break;
					}
				}
				if (!alreadyHave)
					resultPatch += patchData[j];
			}
		}
	}
	return resultPatch;
}

void PatchFusion::multiGraphCut()
{
	int siteNum, labelNum;
	siteNum = patchNum;
	labelNum = matchedModelCategories.size();

	if (labelNum == 1)
	{
		multiGraphCutResult.resize(siteNum, 0);
		return;
	}

	if (siteNum == 1)
	{
		multiGraphCutResult.resize(1);
		int bestLabel = 0;
		double bestDataTerm = std::numeric_limits<double>::max();
		for (size_t i = 0; i < dataTerms[0].size(); ++i)
		{
			if (dataTerms[0][i] < bestDataTerm)
			{
				bestLabel = i;
				bestDataTerm = dataTerms[0][i];
			}
		}
		multiGraphCutResult[0] = bestLabel;
		return;
	}

	GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(siteNum, labelNum);

	for (int i = 0; i < labelNum; ++i)
	{
		for (int j = 0; j < labelNum; ++j)
		{
			if (i != j)
				gc->setSmoothCost(i, j, 1);
		}
	}

	//Set smooth term
	for (size_t i = 0; i < patchConnection.size(); ++i)
	{
		if (patchConnection[i].first < patchConnection[i].second)
		{
			int smoothValue = (int)((patchData[patchConnection[i].first].getPointNum() + patchData[patchConnection[i].second].getPointNum())
					* smoothTerms[patchConnection[i].first][patchConnection[i].second] / 100.0);
			//std::cout << "Smooth Value:" << smoothValue << std::endl;
			gc->setNeighbors(patchConnection[i].first, patchConnection[i].second, smoothValue);
		}
	}

	//Set data term
	for (int i = 0; i < siteNum; ++i)
	{
		for (int j = 0; j < labelNum; ++j)
		{
			if (dataTerms[i][j] > 0)
			{
				int dataValue = (int)(patchData[i].getPointNum() * dataTerms[i][j] / 100.0);
				//std::cout << "Data Value:" << dataValue << std::endl;
				gc->setDataCost(i, j, dataValue);
			}
			else
			{
				gc->setDataCost(i, j, 0);
			}
		}
	}

	gc->swap();
	std::cout << "Energy:" << gc->compute_energy() << std::endl;

	multiGraphCutResult.resize(siteNum, -1);
	for (int i = 0; i < siteNum; ++i)
		multiGraphCutResult[i] = gc->whatLabel(i);

	std::cout << "GC Result" << std::endl;
	for (size_t i = 0; i < multiGraphCutResult.size(); ++i)
	{
		std::cout <<  patchData[i].getLabelIndexes()[0] << " " << matchedModelCategories[multiGraphCutResult[i]] << std::endl;
	}
	std::cout << std::endl;

	delete gc;
}

void PatchFusion::computeDataTerms()
{
	retrieveSinglePatches();
	dataTerms.resize(patchNum);
	for (int i = 0; i < patchNum; ++i)
	{
		dataTerms[i].resize(matchedModelCategories.size(), 1);

		const MatchingResultParser & parser = matchingResultCache[patchData[i].getLabelIndexes()];

		// Null label
		if (parser.matchNum == 0)
		{
				dataTerms[i][0] = 0.3;
		}

		for (int j = 0; j < parser.matchNum; ++j)
		{
			//std::cout << "Patch:" << patchData[i].getLabelIndexes()[0] << " " << parser.modelNames[j] << " " << parser.errors[j] << std::endl;
			auto categoryName = parser.modelNames[j];
			auto found = categoryName.find_first_of("_");
			categoryName = categoryName.substr(0, found);

			auto categoryIter = find(matchedModelCategories.cbegin(), matchedModelCategories.cend(), categoryName);
			if (categoryIter != matchedModelCategories.cend())
			{
				int k = categoryIter - matchedModelCategories.cbegin();
				dataTerms[i][k] = std::min(1 - gaussFunction(parser.errors[j]), dataTerms[i][k]);
			}
		}
		//for (size_t j = 0; j < matchedModelCategories.size(); ++j)
		//	std::cout << "Patch " << patchData[i].getLabelIndexes()[0] << " data term: " << dataTerms[i][j] << std::endl;

		bool isNull = true;
		for (size_t j = 0; j < matchedModelCategories.size(); ++j)
		{
			if (dataTerms[i][j] < 0.5)
				isNull = false;
		}
		if (isNull)
			dataTerms[i][0] = 0.5;
	}
}

void PatchFusion::computeSmoothTerms(const std::vector<unsigned short> & recognizedPatchesByYolo, double probability)
{
	retrieveCoupledPatches();
	for (auto connection : patchConnection)
	{
		int patch1 = connection.first;
		int patch2 = connection.second;

		if (smoothTerms[patch1][patch2] == -1)
		{
			std::vector<unsigned short> labelIndexes = {patchData[patch1].getLabelIndexes()[0], patchData[patch2].getLabelIndexes()[0]};
			std::sort(labelIndexes.begin(), labelIndexes.end());
			const MatchingResultParser & parser = matchingResultCache[labelIndexes];

			double smoothValue = 0;

			for (int j = 0; j < parser.matchNum; ++j)
			{
				smoothValue = std::max(gaussFunction(parser.errors[j]), smoothValue);
			}

			if (probability > 0.5 &&
				std::find(recognizedPatchesByYolo.cbegin(), recognizedPatchesByYolo.cend(), patchData[patch1].getLabelIndexes()[0]) != recognizedPatchesByYolo.cend() &&
				std::find(recognizedPatchesByYolo.cbegin(), recognizedPatchesByYolo.cend(), patchData[patch2].getLabelIndexes()[0]) != recognizedPatchesByYolo.cend())
			{
				smoothTerms[patch1][patch2] = smoothValue * 2 * probability;
				smoothTerms[patch2][patch1] = smoothValue * 2 * probability;
			}
			else
			{
				smoothTerms[patch1][patch2] = smoothValue;
				smoothTerms[patch2][patch1] = smoothValue;
			}
		}
	}
}

void PatchFusion::retrieveSinglePatches()
{
	matchedModelCategories.push_back("null");
	for (int i = 0; i < patchNum; ++i)
	{
		std::vector<int> patchId;
		patchId.push_back(i);

		const MatchingResultParser parser = partialMatchingFacada->getMatchingResult(patchData[i], findAdjacentPatches(patchId));

		std::cout << std::endl;
		std::cout << "Patch: " <<  patchData[i].getLabelIndexes()[0] << " matching result" << std::endl;

		for (int j = 0; j < parser.matchNum; ++j)
		{
			std::cout << parser.modelNames[j] << "  " << parser.errors[j] << std::endl;

			std::string tempName = parser.modelNames[j];
			auto found = tempName.find_first_of("_");
			tempName = tempName.substr(0, found);

			if (find(matchedModelCategories.cbegin(), matchedModelCategories.cend(), tempName) == matchedModelCategories.cend())
			{
				matchedModelCategories.push_back(tempName);
			}
		}
		matchingResultCache.insert({patchData[i].getLabelIndexes(), parser});
		std::cout << std::endl;
	}
}

void PatchFusion::retrieveCoupledPatches()
{
	for (auto connection : patchConnection)
	{
		SegmentedPatch tempPatch(patchData[connection.first]);
		tempPatch += patchData[connection.second];
		if (matchingResultCache.count(tempPatch.getLabelIndexes()) == 0)
		{
			std::vector<int> patchId;
			patchId.push_back(connection.first);
			patchId.push_back(connection.second);

			const MatchingResultParser parser = partialMatchingFacada->getMatchingResult(tempPatch, findAdjacentPatches(patchId));

			std::cout <<std::endl;
			std::cout << "Patch: " <<  patchData[connection.first].getLabelIndexes()[0] << " & " << patchData[connection.second].getLabelIndexes()[0] << " matching result" << std::endl;

			for (int j = 0; j < parser.matchNum; ++j)
			{
				std::cout << parser.modelNames[j] << "  " << parser.errors[j] << std::endl;
			}
			std::cout <<std::endl;

			matchingResultCache.insert({tempPatch.getLabelIndexes(), parser});
		}
	}
}

void PatchFusion::showDataSmoothTerm()
{
	std::cout << std::endl;
	std::cout << "Data Term: ";
	for (size_t i = 0; i < matchedModelCategories.size(); ++i)
	{
		std::cout << matchedModelCategories[i] << " ";
	}
	std::cout << std::endl;

	for (int i = 0; i < patchNum; ++i)
	{
		std::cout << "Patch" << patchData[i].getLabelIndexes()[0] << "  ";
		for (size_t j = 0; j < matchedModelCategories.size(); ++j)
		{
			std::cout << dataTerms[i][j] << "  ";
		}
		std::cout << std::endl;
	}

	std::cout << "Smooth Term:" << std::endl;
	for (size_t i = 0; i < patchConnection.size(); ++i)
	{
		int p1 = patchConnection[i].first;
		int p2 = patchConnection[i].second;
		std::cout << "Patch" << patchData[p1].getLabelIndexes()[0] << " & " << "Patch" << patchData[p2].getLabelIndexes()[0] << "  " << smoothTerms[p1][p2] << std::endl;
	}
	std::cout << std::endl;
}

void PatchFusion::saveDataSmoothTerm(const std::string & filePath)
{
	std::ofstream dataSmoothFile(filePath.c_str());
	for (const auto & modelName : matchedModelCategories)
	{
		dataSmoothFile << modelName << " ";
	}
	dataSmoothFile << std::endl;
	dataSmoothFile << patchNum << std::endl;
	for (auto patchDataTerm : dataTerms)
	{
		for (auto modelDataTerm : patchDataTerm)
		{
			dataSmoothFile << modelDataTerm << " ";
		}
		dataSmoothFile << std::endl;
	}
	for (auto patchSmoothTerm1 : smoothTerms)
	{
		for (auto patchSmoothTerm2 : patchSmoothTerm1)
		{
			dataSmoothFile << patchSmoothTerm2 << " ";
		}
		dataSmoothFile << std::endl;
	}
	dataSmoothFile.close();
}

void PatchFusion::setDataSmoothTerm(const std::string & filePath)
{
	matchedModelCategories.clear();
	std::ifstream dataSmoothFile(filePath.c_str());
	std::string lineStr, tempName;
	std::getline(dataSmoothFile, lineStr);
	std::istringstream tempLine(lineStr);

	while (tempLine >> tempName)
	{
		matchedModelCategories.push_back(tempName);
	}

	int tempNum;
	dataSmoothFile >> tempNum;
	if (tempNum != patchNum)
	{
		std::cerr << "Error: patch num isn't matching." << std::endl;
		return;
	}

	dataTerms.clear();
	dataTerms.resize(patchNum);
	for (int i = 0; i < patchNum; ++i)
	{
		for (size_t j = 0; j < matchedModelCategories.size(); ++j)
		{
			double tempValue;
			dataSmoothFile >> tempValue;
			dataTerms[i].push_back(tempValue);
		}
	}

	smoothTerms.clear();
	smoothTerms.resize(patchNum);
	for (int i = 0; i < patchNum; ++i)
	{
		for (int j = 0; j < patchNum; ++j)
		{
			double tempValue;
			dataSmoothFile >> tempValue;
			smoothTerms[i].push_back(tempValue);
		}
	}
	dataSmoothFile.close();
}

void PatchFusion::generateObjectHypotheses()
{
	// Find object hypothesis that comprised by nearby patches having same label
	std::list<std::list<int>> objectHypoList;
	for (int i = 0; i < patchNum; ++i)
	{
		if (multiGraphCutResult[i] != 0)
		{
			bool isFind = false;
			decltype(objectHypoList.begin()) findPos;
			for (auto listIter1 = objectHypoList.begin(); listIter1 != objectHypoList.end(); ++listIter1)
			{
				for (auto listIter2 = listIter1->begin(); listIter2 != listIter1->end(); ++listIter2)
				{
					if (patchConnectFlags[i][*listIter2] && multiGraphCutResult[i] == multiGraphCutResult[*listIter2])
					{
						if (!isFind)
						{
							isFind = true;
							findPos = listIter1;
							findPos->push_back(i);
							break;
						}
						else
						{
							findPos->insert(findPos->end(), listIter1->begin(), listIter1->end());
							listIter1 = objectHypoList.erase(listIter1);
							--listIter1;
							break;
						}
					}
				}
			}

			if (!isFind)
				objectHypoList.push_back(std::list<int>(1, i));
		}
	}
	
	for (const auto & object : objectHypoList)
	{
		SegmentedPatch tempPatch;
		for (auto patchId : object)
		{
			tempPatch += patchData[patchId];
		}
		objectHypotheses.push_back(tempPatch);

		//if (matchingResultCache.count(tempPatch.getLabelIndexes()) > 0)
		//{
		//	objectHypothesesMatchingResults.push_back(matchingResultCache[tempPatch.getLabelIndexes()]);
		//}
		//else
		//{
		//	/* FIXME: For object hypothesis, needn't to consider adjacent patches <31-07-17, xixia> */
		//	descriptorGenerator.setPointData(tempPatch);
		//	auto patchKeyptRepr = descriptorGenerator.computeDescriptor(keypointNum);
		//	visualWord.assignKeypointsToClusters(patchKeyptRepr);

		//	std::vector<Match> pmResult = partialMatching->searchPartialMatchedModel(patchKeyptRepr);

		//	const MatchingResultParser parser(pmResult, modelData, patchKeyptRepr);

		//	matchingResultCache.insert({tempPatch.getLabelIndexes(), parser});
		//	objectHypothesesMatchingResults.push_back(parser);
		//}
	}
}

void PatchFusion::clearData()
{
	patchData.clear();
	objectHypotheses.clear();
	objectHypothesesMatchingResults.clear();
	patchConnectFlags.clear();
	patchConnection.clear();
	dataTerms.clear();
	smoothTerms.clear();
	multiGraphCutResult.clear();
	matchingResultCache.clear();
	matchedModelCategories.clear();
}

inline double PatchFusion::gaussFunction(double x)
{
	return exp(-pow(x - paraGaussFunctionCenter, 2) / (2 * pow(paraGaussFunctionWidth, 2)));
}
