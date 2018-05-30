#include "FrameSegmentation.h"

FrameSegmentation::~FrameSegmentation()
{

}

void FrameSegmentation::process()
{
	segmentNewFrame();
	//showColoredLabelMap("currLabelMap", currentLabelMap);
	//showColoredLabelMap("projectedLabelMap", projectedLabelMap);

	labelPropagation();
	//showColoredLabelMap("propMap", propagatedLabelMap);
	//cv::waitKey(0);
	updateMergingPairs();
}

void FrameSegmentation::segmentNewFrame()
{
	int rowNum = Resolution::getInstance().rows() / 2;
	int colNum = Resolution::getInstance().cols() / 2;
	cv::Mat edgeMap = cv::Mat::zeros(rowNum, colNum, CV_8UC1);

    for (int i = 0; i < rowNum; ++i)
    {
		for (int j = 0; j < colNum; ++j)
		{
			cv::Vec3f vertex = rawVertexMap.at<cv::Vec3f>(i,j);
			cv::Vec3f normal = rawNormalMap.at<cv::Vec3f>(i,j);

			if (depthMap.at<ushort>(i, j) == 0)
			{
				edgeMap.at<uchar>(i, j) = 0;
				continue;
			}

			float minConcavePenalty = std::numeric_limits<float>::max();
			float maxPointToPlaneDis = -std::numeric_limits<float>::max();
			for(int m = i - 1; m <= i + 1; ++m)
			{
				for(int n = j - 1; n <= j + 1; ++n)
				{
					if(m == i && n == j)
						continue;
					if(m < 0 || n < 0 || m > rowNum - 1 || n > colNum - 1)
						continue;

					cv::Vec3f neighborVertex  = rawVertexMap.at<cv::Vec3f>(m, n);
					cv::Vec3f neighborNormal  = rawNormalMap.at<cv::Vec3f>(m, n);
					float concavePenalty, pointToPlaneDis = normal.dot(neighborVertex - vertex);
					if (pointToPlaneDis > 0)
					{
						concavePenalty = 1;
					}
					else
					{
						concavePenalty = normal.dot(neighborNormal);
					}

					minConcavePenalty = std::min(minConcavePenalty, concavePenalty);
					maxPointToPlaneDis = std::max(maxPointToPlaneDis, std::abs(pointToPlaneDis));
				}
			}
			if (minConcavePenalty > paraConcavityThreshold)
				edgeMap.at<uchar>(i, j) = 255;
			//float axialNoise = 1200 + 19 * (depthMap.at<ushort>(i, j) - 400) * (depthMap.at<ushort>(i, j) - 400);
			float axialNoise = 0.0012 + 0.0019 * pow(depthMap.at<ushort>(i, j) / 1000.0 - 0.4, 2);
			if (maxPointToPlaneDis > axialNoise * 5)
				edgeMap.at<uchar>(i, j) = 0;
		}
	}
   //cv::Mat colorlabel0 = getLabel(edge0);
	//cv::imshow("depthresult1", edgeMap);
	//cv::Mat edgeMap2 = cv::Mat(480, 640, CV_8UC1);
	//for (int i = 0; i < edgeMap2.rows; ++i)
	//{
	//	for (int j = 0; j < edgeMap2.cols; ++j)
	//	{
	//		edgeMap2.at<uchar>(i, j) = edgeMap.at<uchar>(i / 2, j / 2);
	//	}
	//}
	//cv::imwrite("edge2.jpg", edgeMap2);

	//cv::Mat grayMap;
	//cvtColor(rgbMap, grayMap, CV_RGB2GRAY);
	//cv::Mat colorContours1 = computeContours(grayMap,85,50);
	//cv::Mat colorContours2 = computeContours(grayMap,205,40);
	//cv::Mat depthContours = computeContours(edgeMap,115,30);
	//cv::imshow("colorresult1", colorContours1);
	//cv::imshow("colorresult2", colorContours2);
	//cv::imshow("depthresult2", depthContours);

	//for (int i = 2; i < edgeMap.rows - 2; ++i)
	//{
	//	for (int j = 2; j < edgeMap.cols - 2; ++j)
	//	{
	//		if (colorContours1.at<uchar>(i, j) == 0)
	//		{
	//			for (int n = i - 2; n <= i + 2; ++n)
	//			{
	//				for (int m = j - 2; m <= j + 2; ++m)
	//				{
	//					if (edgeMap.at<uchar>(m, n) == 0)
	//					{
	//						edgeMap.at<uchar>(i, j) = 0;
	//						break;
	//					}
	//				}
	//			}
	//		}
	//		if (colorContours2.at<uchar>(i, j) == 0)
	//		{
	//			for (int n = i - 2; n <= i + 2; ++n)
	//			{
	//				for (int m = j - 2; m <= j + 2; ++m)
	//				{
	//					if (edgeMap.at<uchar>(m, n) == 0)
	//					{
	//						edgeMap.at<uchar>(i, j) = 0;
	//						break;
	//					}
	//				}
	//			}
	//		}
	//	}
	//}

	//for (int i = 0; i < depthContours.rows; ++i)
	//{
	//	for (int j = 0; j < depthContours.cols; ++j)
	//	{
	//		if (edgeMap.at<uchar>(i, j) == 255)
	//			depthContours.at<uchar>(i, j) = 255;
	//		else
	//			depthContours.at<uchar>(i, j) = 255 - depthContours.at<uchar>(i, j);
	//	}
	//}
	//cv::imshow("depthresult2", edgeMap);
	//cv::imshow("depthresult3", depthContours);
	//currentLabelMap = getLabel(depthContours);
	currentLabelMap = getLabel(edgeMap);
	//cv::imshow("depthresult4", label);
	//cv::waitKey(0);
}

void FrameSegmentation::labelPropagation()
{
	//showColoredLabelMap("currentLabelMap", currentLabelMap);
	//showColoredLabelMap("projectedLabelMap", projectedLabelMap);
	//int num1, num2, num3, num4;
	//num1 = num2 = num3 = num4 = 0;

	constexpr double pi = 3.141592654;

	std::map<unsigned short, std::map<unsigned short, int>> labelCorrespondences;
	std::map<unsigned short, int> labelCount;
	std::map<unsigned short, unsigned short> labelPropResult;

	for (int i = 0; i < depthMap.rows; ++i)
	{
		for (int j = 0; j < depthMap.cols; ++j)
		{
			unsigned short currentLabel = currentLabelMap.at<ushort>(i, j);
			++labelCount[currentLabel];

			if (currentLabel == 0)
			{
				continue;
			}

			unsigned short projectedLabel = projectedLabelMap.at<ushort>(i, j);
			if (projectedLabel == 0)
			{
				continue;
			}

			//++num4;

			cv::Vec3f rawVertex  = rawVertexMap.at<cv::Vec3f>(i, j);
			cv::Vec3f rawNormal  = rawNormalMap.at<cv::Vec3f>(i, j);
			cv::Vec3f projectedVertex  = projectedVertexMap.at<cv::Vec3f>(i, j);
			cv::Vec3f projectedNormal  = projectedNormalMap.at<cv::Vec3f>(i, j);

			//std::cout << "Raw: " << rawVertex[0] << " " << rawVertex[1] << " " << rawVertex[2] << std::endl;
			//std::cout << "Pro: " << projectedVertex[0] << " " << projectedVertex[1] << " " << projectedVertex[2] << std::endl;

			std::map<unsigned short, int> & tempMap = labelCorrespondences[currentLabel];
            //float sigma = 1200 + 19 * ( depthMap.at<ushort>(r,c) - 400) * (depthMap.at<ushort>(r,c) - 400);
			float axialNoise = 0.0012 + 0.0019 * pow(depthMap.at<ushort>(i, j) / 1000.0 - 0.4, 2);
			float normalThreshold = cos(pi * paraNoramlThreshold / 180);
            //float sigma0 = vec3f_abs(vec3f_dot((vertex-vertexM),vertex));
			
			//if (std::abs(rawVertex.dot(rawVertex-projectedVertex) / norm(rawVertex)) < 10 * axialNoise)
			//	++num1;
			//if (rawNormal.dot(projectedNormal) > normalThreshold)
			//	++num2;

			if (std::abs(rawVertex.dot(rawVertex-projectedVertex) / norm(rawVertex)) < 10 * axialNoise && rawNormal.dot(projectedNormal) > normalThreshold)
			{
				++tempMap[projectedLabel];
				//++num3;
			}
		}
	}

	//std::cout << num1 << " " << num2 << " " << num3 << " " << num4 << std::endl;

	for (const auto & correspondence : labelCorrespondences)
	{
		int totalCount = labelCount[correspondence.first];
		if (totalCount < paraMinPatchSize)
		{
			labelPropResult.insert({correspondence.first, 0});
			continue;
		}

		auto maxIndex = correspondence.second.begin();
		int maxCount = 0;
		std::vector<unsigned short> mergingLabels;
		for (auto corrIter = correspondence.second.begin(); corrIter != correspondence.second.cend(); ++corrIter)
		{
			if (corrIter->second > maxCount)
			{
				maxIndex = corrIter;
				maxCount = corrIter->second;
			}

			if ((float)corrIter->second / totalCount > paraMergingOverlapThreshold)
				mergingLabels.push_back(corrIter->first);
		}

		if ((float)maxCount / totalCount > paraPropagationOverlapThreshold)
		{
			labelPropResult.insert({correspondence.first, maxIndex->first});
			//std::cout << "Label propagation: " << correspondence.first << " -> " << maxIndex->first << std::endl;
		}

		for (size_t i = 0; i < mergingLabels.size(); ++i)
		{
			for (size_t j = i + 1; j < mergingLabels.size(); ++j)
			{
				auto labelPair = mergingLabels[i] < mergingLabels[j] ? std::make_pair(mergingLabels[i], mergingLabels[j]) : std::make_pair(mergingLabels[j], mergingLabels[i]);

				auto mergingCandidatesIter = mergingPairCandidates.find(labelPair); 
				if (mergingCandidatesIter != mergingPairCandidates.cend())
					mergingCandidatesIter->second += 2;
				else
				{
					if (std::find(mergingPairs.cbegin(), mergingPairs.cend(), labelPair) == mergingPairs.cend())
						mergingPairCandidates.insert({labelPair, 1});
				}
			}
		}
	}

	for (int i = 0; i < currentLabelMap.rows; ++i)
	{
		for (int j = 0; j < currentLabelMap.cols; ++j)
		{
			unsigned short currentLabel = currentLabelMap.at<ushort>(i, j);
			if (currentLabel != 0 && labelCount[currentLabel] > paraMinPatchSize && labelPropResult.count(currentLabel) == 0)
			{
				if (labelPool.empty())
					fillLabelPool(50);
				unsigned short distributedLabel = labelPool.front();
				labelPool.pop_front();

				labelPropResult.insert({currentLabel, distributedLabel});
				//std::cout << "Label distribution: " << currentLabel << " -> " << distributedLabel << std::endl;
			}
		}
	}

	propagatedLabelMap = cv::Mat(Resolution::getInstance().rows(), Resolution::getInstance().cols(), CV_16UC1);
	for (int i = 0; i < propagatedLabelMap.rows; ++i)
	{
		for (int j = 0; j < propagatedLabelMap.cols; ++j)
		{
			propagatedLabelMap.at<ushort>(i, j) = labelPropResult[currentLabelMap.at<ushort>(i / 2, j / 2)];
		}
	}
	//cv::pyrUp(propMap, upsampledPropMap, cv::Size(propMap.cols * 2, propMap.rows * 2));
}

void FrameSegmentation::updateMergingPairs()
{
	auto mergingCandidatesIter = mergingPairCandidates.begin();
	while (mergingCandidatesIter != mergingPairCandidates.cend())
	{
		if (mergingCandidatesIter->second > paraMergingCountThreshold && mergingPairs.size() < maxMergingPairSize)
		{
			std::cout << "Label merging: " << mergingCandidatesIter->first.first << " -> " << mergingCandidatesIter->first.second << std::endl;
			mergingPairs.push_back(mergingCandidatesIter->first);
			mergingCandidatesIter = mergingPairCandidates.erase(mergingCandidatesIter);
		}
		else
		{
			mergingCandidatesIter->second = std::max(0, mergingCandidatesIter->second - 1);
			++mergingCandidatesIter;
		}
	}
}

void FrameSegmentation::removeGroundPlainMergingPairs(const std::vector<unsigned short> & labels)
{
	for (auto pairIter = mergingPairs.begin(); pairIter != mergingPairs.end(); )
	{
		bool foundFirst = std::find(labels.cbegin(), labels.cend(), pairIter->first) != labels.cend() ? true : false;
		bool foundSecond = std::find(labels.cbegin(), labels.cend(), pairIter->second) != labels.cend() ? true : false;

		if (foundFirst != foundSecond)
		{
			pairIter = mergingPairs.erase(pairIter);
		}
		else
			++pairIter;
	}
}

bool FrameSegmentation::needMerging() const
{
    return !mergingPairs.empty();
}

void FrameSegmentation::clearMergingPairs()
{
	for (const auto & labelPair : mergingPairs)
		labelPool.push_back(labelPair.second);

	mergingPairs.clear();
	//std::cout << "Merging pairs cleared!" << std::endl;
}

void FrameSegmentation::insertMergingPairs(const std::vector<std::pair<unsigned short, unsigned short>> & pairs)
{
	for (auto pair : pairs)
	{
		if (pair.first > pair.second)
			std::swap(pair.first, pair.second);

		if (std::find(mergingPairs.cbegin(), mergingPairs.cend(), pair) == mergingPairs.cend())
			mergingPairs.push_back(std::move(pair));
	}
}

void FrameSegmentation::recycleOverridedLabels(std::set<unsigned short> && existingLabels)
{
	for (auto label : labelPool)
		existingLabels.insert(label);

	for (int i = 1; i < newLabel; ++i)
	{
		if (existingLabels.count(i) == 0)
			labelPool.push_back(i);
	}
}

cv::Mat FrameSegmentation::getLabel(cv::Mat src)
{    
	int threshval = 150;
	cv::Mat bw = threshval < 128 ? (src < threshval) : (src > threshval);
	cv::Mat labelImage(src.size(), CV_16UC1);
	cv::connectedComponents(bw, labelImage, 4, CV_16U);

	return labelImage;
}

void FrameSegmentation::fillLabelPool(int num)
{
	for (int i = 0; i < num; ++i)
	{
		labelPool.push_back(newLabel);
		++newLabel;
	}
}

void FrameSegmentation::showColoredLabelMap(const std::string & imageName, cv::Mat labelMap)
{
	cv::Mat coloredLabelMap(labelMap.size(), CV_8UC3);
	for (int i = 0; i < coloredLabelMap.rows; ++i)
	{
		for (int j = 0; j < coloredLabelMap.cols; ++j)
		{
			int label = labelMap.at<ushort>(i, j);
			coloredLabelMap.at<cv::Vec3b>(i, j) = cv::Vec3b(label * 0x11 & 0xFF, label * 0x55 & 0xFF, label * 0xDD & 0xFF);
		}
	}
	cv::imshow(imageName.c_str(), coloredLabelMap);
}

cv::Mat FrameSegmentation::computeContours(cv::Mat grayMap, int thresholdValue, size_t minSize)
{
	cv::Mat filteredImage;
	cv::threshold(grayMap, filteredImage, thresholdValue, 255, cv::THRESH_BINARY);
	std::vector<std::vector<cv::Point>> rawContours, filteredContours;
	//CV_CHAIN_APPROX_NONE  获取每个轮廓每个像素点
	cv::findContours(filteredImage, rawContours, CV_RETR_CCOMP, CV_CHAIN_APPROX_TC89_KCOS, cvPoint(0,0));
	//std::cout << contours.size() << std::endl;
	cv::Mat result(grayMap.size(), CV_8UC1, cv::Scalar(255));

	for	(size_t i = 0; i < rawContours.size(); ++i)
	{
		if(rawContours[i].size() > minSize)
		{
			filteredContours.push_back(rawContours[i]);
		}
	}
	std::cout << filteredContours.size() << std::endl;
	cv::drawContours(result, filteredContours, -1, cv::Scalar(0), 1);   // -1 表示所有轮廓
	return result;
}

void FrameSegmentation::compressFrameData()
{
	cv::Mat tempMap;

	tempMap = rgbMap;
	cv::pyrDown(rgbMap, tempMap, cv::Size(rgbMap.cols / 2, rgbMap.rows / 2));
	rgbMap = tempMap;

	tempMap = depthMap;
	cv::pyrDown(depthMap, tempMap, cv::Size(depthMap.cols / 2, depthMap.rows / 2));
	depthMap = tempMap;

	tempMap = rawVertexMap;
	cv::pyrDown(rawVertexMap, tempMap, cv::Size(rawVertexMap.cols / 2, rawVertexMap.rows / 2));
	rawVertexMap = tempMap;

	tempMap = rawNormalMap;
	cv::pyrDown(rawNormalMap, tempMap, cv::Size(rawNormalMap.cols / 2, rawNormalMap.rows / 2));
	rawNormalMap = tempMap;

	//tempMap = projectedVertexMap;
	//cv::pyrDown(projectedVertexMap, tempMap, cv::Size(projectedVertexMap.cols / 2, projectedVertexMap.rows / 2));
	//projectedVertexMap = tempMap;
	tempMap = cv::Mat(Resolution::getInstance().rows() / 2, Resolution::getInstance().cols() / 2, CV_32FC3);
	for (int i = 0; i < tempMap.rows; ++i)
	{
		for (int j = 0; j < tempMap.cols; ++j)
		{
			tempMap.at<cv::Vec3f>(i, j) = localPooling(projectedVertexMap.at<cv::Vec3f>(i * 2, j * 2),
													   projectedVertexMap.at<cv::Vec3f>(i * 2, j * 2 + 1),
													   projectedVertexMap.at<cv::Vec3f>(i * 2 + 1, j * 2),
													   projectedVertexMap.at<cv::Vec3f>(i * 2 + 1, j * 2 + 1));
		}
	}
	projectedVertexMap = tempMap;

	tempMap = cv::Mat(Resolution::getInstance().rows() / 2, Resolution::getInstance().cols() / 2, CV_32FC3);
	for (int i = 0; i < tempMap.rows; ++i)
	{
		for (int j = 0; j < tempMap.cols; ++j)
		{
			tempMap.at<cv::Vec3f>(i, j) = localPooling(projectedNormalMap.at<cv::Vec3f>(i * 2, j * 2),
													   projectedNormalMap.at<cv::Vec3f>(i * 2, j * 2 + 1),
													   projectedNormalMap.at<cv::Vec3f>(i * 2 + 1, j * 2),
													   projectedNormalMap.at<cv::Vec3f>(i * 2 + 1, j * 2 + 1));
		}
	}
	projectedNormalMap = tempMap;

	//tempMap = projectedNormalMap;
	//cv::pyrDown(projectedNormalMap, tempMap, cv::Size(projectedNormalMap.cols / 2,projectedNormalMap.rows / 2));
	//projectedNormalMap= tempMap;

	//cv::pyrDown(projectedLabelMap, tempMap, cv::Size(projectedLabelMap.cols / 2,projectedLabelMap.rows / 2));
	//projectedLabelMap= tempMap;
	tempMap = cv::Mat(Resolution::getInstance().rows() / 2, Resolution::getInstance().cols() / 2, CV_16UC1);
	for (int i = 0; i < tempMap.rows; ++i)
	{
		for (int j = 0; j < tempMap.cols; ++j)
		{
			tempMap.at<ushort>(i, j) = localPooling(projectedLabelMap.at<ushort>(i * 2, j * 2),
													projectedLabelMap.at<ushort>(i * 2, j * 2 + 1),
													projectedLabelMap.at<ushort>(i * 2 + 1, j * 2),
													projectedLabelMap.at<ushort>(i * 2 + 1, j * 2 + 1));
		}
	}
	projectedLabelMap = tempMap;
}

int FrameSegmentation::localPooling(int l1, int l2, int l3, int l4)
{
	if (l1)
		return l1;
	else if (l2)
		return l2;
	else if (l3)
		return l3;
	else
		return l4;
}

cv::Vec3f FrameSegmentation::localPooling(cv::Vec3f v1, cv::Vec3f v2, cv::Vec3f v3, cv::Vec3f v4)
{
	if (norm(v1))
		return v1;
	else if (norm(v2))
		return v2;
	else if (norm(v3))
		return v3;
	else
		return v4;
}

const unsigned short * FrameSegmentation::getPropagatedLabelMap() const
{
	return reinterpret_cast<unsigned short *>(propagatedLabelMap.data);
}

const std::vector<std::pair<unsigned short, unsigned short>> & FrameSegmentation::getMergingPairs() const
{
	return mergingPairs;
}

unsigned short FrameSegmentation::getCurrentLabelRange() const
{
	return newLabel;
}

void FrameSegmentation::setFrameData(const unsigned char * rgb,
									 const unsigned short * depth,
									 const float * rawVertexes,
									 const float * rawNormals,
									 const float * projectedVertexes,
									 const float * projectedNormals,
									 const unsigned short * projectedLabels)
{
	rgbMap = cv::Mat(Resolution::getInstance().rows(), Resolution::getInstance().cols(), CV_8UC3, const_cast<unsigned char *>(rgb));
	depthMap = cv::Mat(Resolution::getInstance().rows(), Resolution::getInstance().cols(), CV_16UC1, const_cast<unsigned short *>(depth));
	rawVertexMap = cv::Mat(Resolution::getInstance().rows(), Resolution::getInstance().cols(), CV_32FC3, const_cast<float *>(rawVertexes));
	rawNormalMap = cv::Mat(Resolution::getInstance().rows(), Resolution::getInstance().cols(),  CV_32FC3, const_cast<float *>(rawNormals));
	projectedVertexMap = cv::Mat(Resolution::getInstance().rows(), Resolution::getInstance().cols(),  CV_32FC3, const_cast<float *>(projectedVertexes));
	projectedNormalMap = cv::Mat(Resolution::getInstance().rows(), Resolution::getInstance().cols(), CV_32FC3, const_cast<float *>(projectedNormals));
	projectedLabelMap = cv::Mat(Resolution::getInstance().rows(), Resolution::getInstance().cols(),  CV_16UC1, const_cast<unsigned short *>(projectedLabels));

	compressFrameData();
}
