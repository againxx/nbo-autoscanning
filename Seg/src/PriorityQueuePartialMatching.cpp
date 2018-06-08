#include "PriorityQueuePartialMatching.h"

PriorityQueuePartialMatching::PriorityQueuePartialMatching(const std::vector<KeypointRepresentation> & models)
 : PartialMatching(models),
   bestPartialMatch(nullptr)
{

}

PriorityQueuePartialMatching::~PriorityQueuePartialMatching()
{
	if (bestPartialMatch)
		delete [] bestPartialMatch;
}

std::vector<Match> PriorityQueuePartialMatching::searchPartialMatchedModel(const KeypointRepresentation & patch)
{
	patchData = patch;

	// Clear data
	while (!matchPriorityQueue.empty())
		matchPriorityQueue.pop();
	if (bestPartialMatch)
		delete [] bestPartialMatch;
	bestPartialMatch = new Match[modelData.size()];
	correspondPairs.clear();
	correspondPairs.resize(modelData.size());
	resultMatches.clear();

	// Do priority driven search
	auto startTime = std::chrono::system_clock::now();
	priorityDrivenSearch();
	auto endTime = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	std::cout << "Spend " << (double)duration.count() * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den  << " seconds doing priority driven search." << std::endl;

	showResult();
	return resultMatches;
}

void PriorityQueuePartialMatching::showResult()
{
	for (size_t i = 0; i < resultMatches.size(); ++i)
	{
		std::cout << modelData[resultMatches[i].modelIndex].getName() << std::endl;
		std::cout << "Matching cost:" << resultMatches[i].costValue << std::endl;
		std::cout << "Matched Points:" << std::endl;
		for (int j = 0; j < resultMatches[i].pairNum; ++j)
		{
			std::cout << modelData[resultMatches[i].modelIndex].getPositions()[resultMatches[i].modelKeypointIndexes[j]][0] << " ";
			std::cout << modelData[resultMatches[i].modelIndex].getPositions()[resultMatches[i].modelKeypointIndexes[j]][1] << " ";
			std::cout << modelData[resultMatches[i].modelIndex].getPositions()[resultMatches[i].modelKeypointIndexes[j]][2] << " ";
			std::cout << patchData.getPositions()[resultMatches[i].patchKeypointIndexes[j]][0] << " ";
			std::cout << patchData.getPositions()[resultMatches[i].patchKeypointIndexes[j]][1] << " ";
			std::cout << patchData.getPositions()[resultMatches[i].patchKeypointIndexes[j]][2] << std::endl;
		}
	}
}

void PriorityQueuePartialMatching::priorityDrivenSearch()
{
	omp_lock_t lock;
	omp_init_lock(&lock);
	//auto startTime = std::chrono::system_clock::now();
	int bigNum = 0;
	int smallNum = 0;
	int diameterNum = 0;
	// Create correspondences
	std::cout << "Diameter:" << patchData.getDiameter()<< std::endl;
	auto startTime = std::chrono::system_clock::now();
	#pragma omp parallel for
	for (size_t i = 0; i < modelData.size(); ++i)	//target in datebase
	{
		if (modelData[i].getDiameter() < 0.6 * patchData.getDiameter())
		{
			omp_set_lock(&lock);
			diameterNum++;	
			omp_unset_lock(&lock);
			continue;
		}
		for (unsigned int j = 0; j < patchData.getNum(); ++j)		//query feature
		{
			for (unsigned int k = 0; k < modelData[i].getNum(); ++k)	//target feature
			{
				Match tempMatch;
				tempMatch.pairNum = 1;
				tempMatch.modelIndex = i;
				tempMatch.modelKeypointIndexes.push_back(k);
				tempMatch.patchKeypointIndexes.push_back(j);
				computeCost(tempMatch);
				if (tempMatch.costValue < 99999)
				{
					omp_set_lock(&lock);
					bigNum++;
					omp_unset_lock(&lock);
					//std::cout << tempMatch.costValue << " " << modelName[i] << std::endl;
				}
				if (tempMatch.costValue < paraOnePairThreshold)
				{
					omp_set_lock(&lock);
					smallNum++;
					matchPriorityQueue.push(tempMatch);
					correspondPairs[i].push_back(tempMatch);
					if (bestPartialMatch[i].costValue > tempMatch.costValue)
						bestPartialMatch[i] = tempMatch;
					omp_unset_lock(&lock);
				}
			}
		}
	}

	/*std::vector<int> validModelIndex;
	for (int i = 0; i < modelNum; ++i)
	{
		if (modelDiameter[i] < 0.6 * descGenerator.diameter)
		{
			diameterNum++;
			continue;
		}
		validModelIndex.push_back(i);
	}

	float * validModelDescriptor = new float[validModelIndex.size() * modelKeypointNum * descriptorSize];
	for (size_t i = 0; i < validModelIndex.size(); ++i)
	{
		memcpy(validModelDescriptor + i * modelKeypointNum * descriptorSize, modelKeypointDescriptor[validModelIndex[i]], modelKeypointNum * descriptorSize * sizeof(float));
	}

	double * correspondCost = new double[validModelIndex.size() * modelKeypointNum * patchKeypointNum];
	computeCorrespondPair(descGenerator.keypointDescriptor, validModelDescriptor, validModelIndex.size(), patchKeypointNum, modelKeypointNum, descriptorSize, 
			paraMaxRank, paraCoefRank, paraExpRank, correspondCost);

	for (size_t i = 0; i < validModelIndex.size(); ++i)
	{
		for (int j = 0; j < modelKeypointNum; ++j)
		{
			for (int k = 0; k < patchKeypointNum; ++k)
			{
				Match tempMatch;
				tempMatch.pairNum = 1;
				tempMatch.modelIndex = validModelIndex[i];
				tempMatch.modelKeypointIndex.push_back(j);
				tempMatch.patchKeypointIndex.push_back(k);
				tempMatch.costNum = 1;
				tempMatch.costValue = correspondCost[i * modelKeypointNum * patchKeypointNum + j * patchKeypointNum + k];
				if (tempMatch.costValue < paraOnePairThreshold)
				{
					smallNum++;
					matchPriorityQueue.push(tempMatch);
					correspondPairs[validModelIndex[i]].push_back(tempMatch);
					if (bestPartialMatch[validModelIndex[i]].costValue > tempMatch.costValue)
						bestPartialMatch[validModelIndex[i]] = tempMatch;
				}
			}
		}
	}
	delete [] validModelDescriptor;
	delete [] correspondCost;*/
	auto endTime = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	std::cout << "Spend " << (double)duration.count() * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den  << " seconds computing correspondence." << std::endl;
	

	//endTime = std::chrono::system_clock::now();
	//duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	//std::cout << "Spend " << (double)duration.count() * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den  << " seconds computing correspondence." << std::endl;


	std::cout << smallNum << " " << diameterNum << std::endl;

	// Expand matches until find complete ones
	int completeMatchCount = 0;
	int attemptCount = 0;
	int priorPairNum = 0;
	while (completeMatchCount < paraCandidateCount && matchPriorityQueue.size())
	{
		// Pop best partial match off priority queue
		Match bMatch = matchPriorityQueue.top();
		matchPriorityQueue.pop();
		size_t target = bMatch.modelIndex;
		std::cout << "Currently best match has " << bMatch.pairNum << " corresponded features, cost is " << bMatch.costValue 
			<< ", name is " << bMatch.modelIndex << " " << modelData[bMatch.modelIndex].getName() << std::endl;

		if (priorPairNum == bMatch.pairNum)
		{
			attemptCount++;
		}
		else
		{
			priorPairNum = bMatch.pairNum;
		}
		// Check for complete match (num of correspondence pairs >= paraCorrespondCount)
		if (bMatch.pairNum >= paraCorrespondCount || attemptCount > paraMaxAttemptCount)
		{
			if (bMatch.pairNum >= 3)
			{
				resultMatches.push_back(bMatch);
				completeMatchCount++;
			}
			removeMatchesFromPriorityQueue(target);
			attemptCount = 0;
			continue;
		}

		// Extend match
		//Match bestExtendMatch;
		#pragma omp parallel for
		for (size_t i = 0; i < correspondPairs[target].size(); ++i)
		{
			Match tempMatch = extendMatch(bMatch, correspondPairs[target][i]);
			if (tempMatch.costValue < paraMultiPairThreshold)
			{
				omp_set_lock(&lock);
				matchPriorityQueue.push(tempMatch);
				if (bestPartialMatch[target].costValue > tempMatch.costValue)
					bestPartialMatch[target] = tempMatch;
				omp_unset_lock(&lock);
				//if (bestExtendMatch.costValue > tempMatch.costValue)
					//bestExtendMatch = tempMatch;
			}
		}
		/*if (bestExtendMatch.costValue < paraMultiPairThreshold)
		{
			matchPriorityQueue.push(bestExtendMatch);
			if (bestPartialMatch[target].costValue > bestExtendMatch.costValue)
				bestPartialMatch[target] = bestExtendMatch;
		}*/
		//if (bestPartialMatch[target].costValue > bestExtendMatch.costValue || bestPartialMatch[target].pairNum < bestExtendMatch.pairNum)
			//bestPartialMatch[target] = bestExtendMatch;
		std::cout << "Extend finished, current queue size:" << matchPriorityQueue.size() << std::endl;
	}
	for (size_t i = 0; i < modelData.size(); ++i)
	{
		bool alreadyExist = false;
		std::vector<Match> tempResultMatches;
		for (size_t j = 0; j < resultMatches.size(); ++j)
		{
			if (resultMatches[j].modelIndex == i)
			{
				alreadyExist = true;
				break;
			}
			if (resultMatches[j].pairNum > bestPartialMatch[i].pairNum || (resultMatches[j].pairNum == bestPartialMatch[i].pairNum && resultMatches[j].costValue <= bestPartialMatch[i].costValue))
			{
				tempResultMatches.push_back(resultMatches[j]);
			}
		}
		if (alreadyExist)
		{
			continue;
		}
		else
		{
			if ((int)tempResultMatches.size() < paraCandidateCount && bestPartialMatch[i].pairNum >= 3)
				tempResultMatches.push_back(bestPartialMatch[i]);
			resultMatches = tempResultMatches;
		}
	}
	omp_destroy_lock(&lock);
}

void PriorityQueuePartialMatching::computeCost(Match & m)
{
	size_t target = m.modelIndex;

	// When there is only one pair feature correspondence, we only have three terms in cost function. Otherwise, five terms.
	if (m.pairNum == 1)
	{
		// Cradius
		/*double radiusCost = 0;
		float radius = (modelKeypointPosition[target][m.modelKeypointIndex[0]] - modelCenterPoint[target]).norm();
		radiusCost = radius / modelAverageRadius[target];

		radius = (descGenerator.keypointPosition[m.patchKeypointIndex[0]] - descGenerator.centerPoint).norm();
		radiusCost = abs(radiusCost - radius / descGenerator.averageRadius);
		//std::cout << "Radius Cost:" << radiusCost << std::endl;

		if (radiusCost > paraMaxRadius)			// Throw away obviously poor feature correspondence
		{
			m.costValue = std::numeric_limits<double>::max();
			return;
		}

		// Cnormal
		double normalCost = 0;
		Eigen::Vector3f vRadius = (modelKeypointPosition[target][m.modelKeypointIndex[0]] - modelCenterPoint[target]).normalized();
		normalCost = abs(vRadius.dot(modelKeypointNormal[target][m.modelKeypointIndex[0]]));

		vRadius = (descGenerator.keypointPosition[m.patchKeypointIndex[0]] - descGenerator.centerPoint).normalized();
		normalCost = abs(normalCost - abs(vRadius.dot(descGenerator.keypointNormal[m.patchKeypointIndex[0]])));
		//std::cout << "Normal Cost:" << normalCost << std::endl;

		if (normalCost > paraMaxNormal)			// Throw away obviously poor feature correspondence
		{
			m.costValue = std::numeric_limits<double>::max();
			return;
		}*/

		//	Crank
		double rankCost = 0;
		int descriptorSize = KeypointRepresentation::descriptorSize;
		for (int i = 0; i < descriptorSize; ++i)
		{
			float f1 = modelData[target].getDescriptors()[m.modelKeypointIndexes[0] * descriptorSize + i];
			float f2 = patchData.getDescriptors()[m.patchKeypointIndexes[0] * descriptorSize + i];
			rankCost += pow(f1 - f2, 2);
		}
		rankCost = sqrt(rankCost);

		if (rankCost > paraMaxRank)				// Throw away obviously poor feature correspondence
		{
			m.costValue = std::numeric_limits<double>::max();
			return;
		}

		m.costValue = paraCoefRank * pow(rankCost, paraExpRank); //+ paraCoefRadius * pow(radiusCost, paraExpRadius) + paraCoefNormal * pow(normalCost, paraExpNormal);
		m.costTermNum = 1;
		m.costValue /= m.costTermNum;
	}
	else			// Only cost of the new pair of features needed to compute
	{
		double cost = 0;
		/*// Cradius
		double radiusCost = 0;
		float radius = (modelKeypointPosition[target][m.modelKeypointIndex.back()] - modelCenterPoint[target]).norm();
		radiusCost = radius / modelAverageRadius[target];

		radius = (descGenerator.keypointPosition[m.patchKeypointIndex.back()] - descGenerator.centerPoint).norm();
		radiusCost = abs(radiusCost - radius / descGenerator.averageRadius);

		if (radiusCost > paraMaxRadius)			// Throw away obviously poor feature correspondence
		{
			m.costValue = std::numeric_limits<double>::max();
			return;
		}

		// Cnormal
		double normalCost = 0;
		Eigen::Vector3f vRadius = (modelKeypointPosition[target][m.modelKeypointIndex.back()] - modelCenterPoint[target]).normalized();
		normalCost = abs(vRadius.dot(modelKeypointNormal[target][m.modelKeypointIndex.back()]));

		vRadius = (descGenerator.keypointPosition[m.patchKeypointIndex.back()] - descGenerator.centerPoint).normalized();
		normalCost = abs(normalCost - abs(vRadius.dot(descGenerator.keypointNormal[m.patchKeypointIndex.back()])));

		if (normalCost > paraMaxNormal)			// Throw away obviously poor feature correspondence
		{
			m.costValue = std::numeric_limits<double>::max();
			return;
		}*/

		//	Crank
		/*double rankCost = 0;
		for (int i = 0; i < descriptorSize; ++i)
		{
			float f1 = modelKeypointDescriptor[target][m.modelKeypointIndex.back() * descriptorSize + i];
			float f2 = descGenerator.keypointDescriptor[m.patchKeypointIndex.back() * descriptorSize + i];
			rankCost += pow(f1 - f2, 2);
		}
		rankCost = sqrt(rankCost);

		if (rankCost > paraMaxRank)				// Throw away obviously poor feature correspondence
		{
			m.costValue = std::numeric_limits<double>::max();
			return;
		}

		cost = paraCoefRank * pow(rankCost, paraExpRank); //+ paraCoefRadius * pow(radiusCost, paraExpRadius) + paraCoefNormal * pow(normalCost, paraExpNormal);*/
	
		double lengthCost = 0;
		double orientCost = 0;
		for (int i = 0; i < m.pairNum - 1; ++i)
		{
			// Clength
			float l1 = (modelData[target].getPositions()[m.modelKeypointIndexes.back()] - modelData[target].getPositions()[m.modelKeypointIndexes[i]]).norm();
			float l2 = (patchData.getPositions()[m.patchKeypointIndexes.back()] - patchData.getPositions()[m.patchKeypointIndexes[i]]).norm();
			paraMinLength = 0.3 * std::min(modelData[target].getAverageRadius(), patchData.getAverageRadius());
			if (l1 < paraMinLength || l2 < paraMinLength)		// Avoid matches comprised of features in close proximity to one another
			{
				m.costValue = std::numeric_limits<double>::max();
				return;
			}

			//lengthCost = abs(l1 / modelAverageLength[target] - l2 / descGenerator.averageLength) / max(l1 / modelAverageLength[target], l2 / descGenerator.averageLength);
			lengthCost = std::abs(l1 - l2) / std::max(l1, l2);

			if (lengthCost > paraMaxLength)				// Throw away obviously poor feature correspondence
			{
				m.costValue = std::numeric_limits<double>::max();
				return;
			}

			// Corient
			Eigen::Vector3f v1 = (modelData[target].getPositions()[m.modelKeypointIndexes.back()] - modelData[target].getPositions()[m.modelKeypointIndexes[i]]).normalized();
			Eigen::Vector3f v2 = (patchData.getPositions()[m.patchKeypointIndexes.back()] - patchData.getPositions()[m.patchKeypointIndexes[i]]).normalized();

			orientCost = std::abs(std::abs(modelData[target].getNormals()[m.modelKeypointIndexes.back()].dot(modelData[target].getNormals()[m.modelKeypointIndexes[i]]))
					   - std::abs(patchData.getNormals()[m.patchKeypointIndexes.back()].dot(patchData.getNormals()[m.patchKeypointIndexes[i]])));
			orientCost += std::abs(std::abs(v1.dot(modelData[target].getNormals()[m.modelKeypointIndexes.back()]))
					    - std::abs(v2.dot(patchData.getNormals()[m.patchKeypointIndexes.back()])));
			orientCost += std::abs(std::abs(v1.dot(modelData[target].getNormals()[m.modelKeypointIndexes[i]]))
						- std::abs(v2.dot(patchData.getNormals()[m.patchKeypointIndexes[i]])));

			if (orientCost > paraMaxOrient)
			{
				m.costValue = std::numeric_limits<double>::max();
				return;
			}

			cost += paraCoefLength * pow(lengthCost, paraExpLength) + paraCoefOrient * pow(orientCost, paraExpOrient);
		}
		m.costValue += cost;
		m.costTermNum += (1 + 2 * (m.pairNum - 1));
		m.costValue /= m.costTermNum;
	}
}

void PriorityQueuePartialMatching::removeMatchesFromPriorityQueue(size_t target)
{
	std::priority_queue<Match, std::vector<Match>, MatchComparison> clearedPriorityQueue;
	while (!matchPriorityQueue.empty())
	{
		Match tempMatch = matchPriorityQueue.top();
		matchPriorityQueue.pop();
		if (tempMatch.modelIndex != target)
			clearedPriorityQueue.push(tempMatch);
	}
	matchPriorityQueue = clearedPriorityQueue;
}

Match PriorityQueuePartialMatching::extendMatch(const Match & match, const Match & pair)
{
	Match newMatch = match;
	for (int i = 0; i < newMatch.pairNum; ++i)
	{
		if (newMatch.modelKeypointIndexes[i] == pair.modelKeypointIndexes[0] && newMatch.patchKeypointIndexes[i] == pair.patchKeypointIndexes[0])		//pair already exist in match, couldn't extend
		{
			newMatch.costValue = std::numeric_limits<double>::max();
			return newMatch;
		}
	}
	// Extend match with new pair of features
	newMatch.pairNum++;
	newMatch.modelKeypointIndexes.push_back(pair.modelKeypointIndexes[0]);
	newMatch.patchKeypointIndexes.push_back(pair.patchKeypointIndexes[0]);
	newMatch.costValue *= newMatch.costTermNum;
	newMatch.costValue += pair.costValue;
	computeCost(newMatch);			//update cost
	return newMatch;
}
