#include "ScanningScheme.h"

ScanningScheme::ScanningScheme(const std::string & datasetDir, const std::string & plyDir)
 : lastNBOPosition(0, 0, 0),
   partialMatchingFacada(std::make_shared<PartialMatchingFacada>(datasetDir)),
   patchFusion(partialMatchingFacada),
   nbo(partialMatchingFacada),
   modelsPlyDir(plyDir.back() == '/' ? plyDir : plyDir + '/'),
   drawProgram(loadProgramFromFile("draw_matched_models.vert", "draw_matched_models.frag"))
{

}

ScanningScheme::ScanningScheme(const std::string & datasetDir, const std::string & visualWordFilePath, const std::string & plyDir)
 : lastNBOPosition(0, 0, 0),
   partialMatchingFacada(std::make_shared<PartialMatchingFacada>(datasetDir, visualWordFilePath)),
   patchFusion(partialMatchingFacada),
   nbo(partialMatchingFacada),
   modelsPlyDir(plyDir.back() == '/' ? plyDir : plyDir + '/'),
   drawProgram(loadProgramFromFile("draw_matched_models.vert", "draw_matched_models.frag"))
{

}

ScanningScheme::~ScanningScheme()
{
	if (glIsBuffer(matchedModelsVBO))
		glDeleteBuffers(1, &matchedModelsVBO);
}

void ScanningScheme::process(int pointNum,
							 Eigen::Vector4f * mapData,
							 float confidence,
							 const Eigen::Matrix4f & camPose,
							 std::vector<unsigned short> recognizedPatchesByYolo,
							 double probability)
{
	mtx.lock();
	processing = true;
	mtx.unlock();

	decomposeMapIntoPatches(pointNum, mapData, confidence);

	process(camPose, recognizedPatchesByYolo, probability);
}

void ScanningScheme::process(const Eigen::Matrix4f & camPose,
							 std::vector<unsigned short> recognizedPatchesByYolo,
							 double probability)
{
	mtx.lock();
	processing = true;
	mtx.unlock();
	if (patchData.empty())
	{
		std::cerr << "Error: patch data is empty!" << std::endl;
		return;
	}

	removeInappropriatePatches(camPose);

	processNBO(camPose, recognizedPatchesByYolo, probability);

	if (nbo.hasNBO())
	{
		NBV nbv(nbo.getNBO(), nboMatchingModels, nbo.getNBOMatchingResult().errors);
		nbvGoalsQueue = nbv.getNBVQueue(camPose);

		goalExistence = true;

		mtx.lock();
		processing = false;
		mtx.unlock();
	}
	else
	{
		std::cout << std::endl;
		std::cout << "Search far patches!!!" << std::endl;
		std::cout << std::endl;

		processNBO(camPose, recognizedPatchesByYolo, probability, true);

		if (nbo.hasNBO())
		{
			NBV nbv(nbo.getNBO(), nboMatchingModels, nbo.getNBOMatchingResult().errors);
			nbvGoalsQueue = nbv.getNBVQueue(camPose);

			goalExistence = true;

			mtx.lock();
			processing = false;
			mtx.unlock();
		}
		else
		{
			std::cout << "Not found next-best-object!" << std::endl;

			mtx.lock();
			processing = false;
			mtx.unlock();
		}
	}

	std::cout << "\nTotally recognize " << recognizedObjects.size() << " objects!\n" << std::endl;
}

void ScanningScheme::processNBO(const Eigen::Matrix4f & camPose,
								const std::vector<unsigned short> & recognizedPatchesByYolo,
								double probability,
								bool searchFar)
{
	if (searchFar)
		patchFusion.setPatchData(farPatches);
	else
		patchFusion.setPatchData(patchData);

	patchFusion.process(recognizedPatchesByYolo, probability);

	nbo.computeNBO(camPose, patchFusion.getObjectHypotheses());

	mtx.lock();
	insertRecognizedObjects(nbo.getRecognizedObjects());
	mtx.unlock();

	if (!nbo.hasNBO())
		return;

	updateBlacklist(nbo.getNBO().getCenterPosition());

	loadNBOMatchingModels(nbo.getNBOMatchingResult().modelNames, nbo.getNBOMatchingResult().transformationMatrixs);
}

void ScanningScheme::loadPatchData(const std::string & patchDirPath)
{
	patchData.clear();

	DirectoryReader dirReader(patchDirPath);
	if (dirReader.isEmpty())
	{
		std::cerr << "Error: couldn't read patch directory!" << std::endl;
		return;
	}

	std::string fileName, filePath;

	// Load model's keypoints files in dir
	while (dirReader.getNextFile(fileName, filePath))
	{
		patchData.emplace_back(filePath);
		patchData.back().setLabelIndexes(patchData.size());
	}
	std::cout << "Successfully loaded " << patchData.size() << " patches!" << std::endl;
}

void ScanningScheme::clearGoal()
{
	while (!nbvGoalsQueue.empty())
		nbvGoalsQueue.pop();

	nboMatchingModels.clear();
	totalModelsPointNum = 0;
	goalExistence = false;

	glDeleteBuffers(1, &matchedModelsVBO);
}

void ScanningScheme::clearRecognizedUnmergedPatches()
{
	recognizedUnmergedPatchLabels.clear();
	mtx.unlock();
}

void ScanningScheme::decomposeMapIntoPatches(int pointNum, Eigen::Vector4f * mapData, float confidence)
{
	patchData.clear();

	std::map<unsigned short, Patch> patches;
	for (int i = 0; i < pointNum; ++i)
	{
		Eigen::Vector4f pos = mapData[3 * i];
		if (pos[3] > confidence)
		{
			Eigen::Vector4f col = mapData[3 * i + 1];
			int label = static_cast<int>(col[1]) >> 8 & 0xFFFF;
			if (label != 0)
				++patches[label].num;
		}
	}

	for (auto & patch : patches)
	{
		int patchPointNum = patch.second.num;
		if (patch.second.num > paraMinPatchPointNum)
		{
			patch.second.positions = new Eigen::Vector3f[patchPointNum];
			patch.second.colors = new Eigen::Vector3f[patchPointNum];
			patch.second.normals = new Eigen::Vector3f[patchPointNum];
			patch.second.radii = new float[patchPointNum];
			patch.second.index = 0;
		}
	}

	for (int i = 0; i < pointNum; ++i)
	{
		Eigen::Vector4f pos = mapData[3 * i];
		if (pos[3] > confidence)
		{
			Eigen::Vector4f col = mapData[3 * i + 1];
			int label = static_cast<int>(col[1]) >> 8 & 0xFFFF;
			if (label != 0 && patches[label].num > paraMinPatchPointNum)
			{
				Eigen::Vector4f nor = mapData[3 * i + 2];
				int index = patches[label].index;
				patches[label].positions[index] = pos.head(3);

				Eigen::Vector3f color;
				color[0] = static_cast<int>(col[0]) >> 16 & 0xFF;
				color[1] = static_cast<int>(col[0]) >> 8 & 0xFF;
				color[2] = static_cast<int>(col[0]) & 0xFF;
				patches[label].colors[index] = color;

				patches[label].normals[index] = nor.head(3);

				patches[label].radii[index] = nor[3];

				++patches[label].index;
			}
		}
	}

	// Since we use shared_ptr in SegmentedPatch, there is no need delete patch's property
	for (auto & patch : patches)
	{
		if (patch.second.num > paraMinPatchPointNum)
		{
			patchData.push_back(SegmentedPatch(patch.second.num,
											   patch.second.positions,
											   patch.second.colors,
											   patch.second.normals,
											   patch.second.radii,
											   patch.first));

			// Update recognized patch data
			if (recognizedObjects.find(patch.first) != recognizedObjects.cend())
				recognizedObjects[patch.first] = patchData.back();
		}
	}

	delete [] mapData;
}

void ScanningScheme::removeInappropriatePatches(const Eigen::Matrix4f & camPose)
{
	farPatches.clear();
	findGroundPlainPatches();

	Eigen::Vector3f camPosition = camPose.topRightCorner(3,1);
	std::vector<SegmentedPatch> filteredPatches;
	for (const auto & patch : patchData)
	{
		if (!isGroundPlainPatch(patch) &&
			!isRecognizedPatch(patch) &&
			!isBlacklistedPatch(patch) &&
			patch.getBoundingBox().getDiameter() < paraMaxPatchDiameter)
		{
			if ((patch.getCenterPosition() - camPosition).norm() < paraFarPatchDis)
				filteredPatches.push_back(patch);
			else
				farPatches.push_back(patch);
		}
		else
			std::cout << "Filtered patch: " << patch.getLabelIndexes()[0] << std::endl;
	}
	patchData = filteredPatches;
}

bool ScanningScheme::isRecognizedPatch(const SegmentedPatch & patch)
{
	if (recognizedObjects.find(patch.getLabelIndexes()[0]) != recognizedObjects.cend())
		return true;

	for (const auto & recognizedObject : recognizedObjects)
	{
		if (isAdjacent(recognizedObject.second, patch))
		{
			SegmentedPatch tempPatch = recognizedObject.second + patch;
			const MatchingResultParser parser = partialMatchingFacada->getMatchingResult(tempPatch, true); 
			for (auto error : parser.errors)
			{
				if (error < paraMaxErrorConsiderRecognized * tempPatch.getBoundingBox().getDiameter())
				{
					std::vector<unsigned short> patchLabels{recognizedObject.first, patch.getLabelIndexes()[0]};
					mtx.lock();
					recognizedUnmergedPatchLabels.push_back(patchLabels);
					mtx.unlock();
					return true;
				}
			}
		}
	}

	return false;
}

bool ScanningScheme::isBlacklistedPatch(const SegmentedPatch & patch)
{
	for (const auto & position : positionBlacklist)
	{
		if ((patch.getCenterPosition() - position).norm() < paraBlacklistDistanceThreshold)
			return true;
	}
	return false;
}

bool ScanningScheme::isGroundPlainPatch(const SegmentedPatch & patch)
{
	if (std::find(groundPlainLabels.cbegin(), groundPlainLabels.cend(), patch.getLabelIndexes()[0]) == groundPlainLabels.cend())
		return false;
	else
		return true;
}

void ScanningScheme::findGroundPlainPatches()
{
	groundPlainLabels.clear();
	int maxPointNum = 0;
	int largestPatchIndex = 0;
	for (const auto & patch : patchData)
	{
		if (patch.getPointNum() > maxPointNum)
		{
			maxPointNum = patch.getPointNum();
			largestPatchIndex = patch.getLabelIndexes()[0];
		}
		std::cout << "Patch " << patch.getLabelIndexes()[0] << std::endl;
		std::cout << "y max: " << patch.getBoundingBox().yMax << " y min: " << patch.getBoundingBox().yMin << std::endl;
		if ((patch.getBoundingBox().yMin + patch.getBoundingBox().yMax) / 2 > 0.95 && patch.getBoundingBox().yMax - patch.getBoundingBox().yMin < 0.1)
		{
			groundPlainLabels.push_back(patch.getLabelIndexes()[0]);
		}
	}

	if (std::find(groundPlainLabels.cbegin(), groundPlainLabels.cend(), largestPatchIndex) == groundPlainLabels.cend())
		groundPlainLabels.push_back(largestPatchIndex);

	std::cout << std::endl;
	for (auto index : groundPlainLabels)
	{
		std::cout << "Ground plain patch: " << index << std::endl;
	}
	std::cout << std::endl;

}

void ScanningScheme::insertRecognizedObjects(const std::vector<SegmentedPatch> & objects)
{
	for (const auto & object : objects)
	{
		recognizedObjects[object.getLabelIndexes()[0]] = object;
		recognizedUnmergedPatchLabels.push_back(object.getLabelIndexes());
	}
}

void ScanningScheme::updateBlacklist(const Eigen::Vector3f & nboCenterPosition)
{
	if ((nboCenterPosition - lastNBOPosition).norm() < paraBlacklistDistanceThreshold)
	{
		++nboCounter;
		if (nboCounter >= paraBlacklistCountThreshold)
			positionBlacklist.push_back(nboCenterPosition);
	}
	else
	{
		nboCounter = 0;
	}
	lastNBOPosition = nboCenterPosition;
}

void ScanningScheme::loadNBOMatchingModels(const std::vector<std::string> & modelNames, const std::vector<Eigen::Matrix4f> & transMats)
{
	/* TODO: using transformation matrix from model to patch <22-10-17, xiaxi> */
	for (size_t i = 0; i < modelNames.size(); ++i)
	{
		std::string modelFilePath = modelsPlyDir + modelNames[i] + ".ply";
		nboMatchingModels.push_back(SegmentedPatch(modelFilePath) * transMats[i]);
	}
	alreadyCopy = false;
}

void ScanningScheme::copyNBOMatchingModelsIntoGL()
{
	totalModelsPointNum = 0;
	for (const auto & patch : nboMatchingModels)
	{
		totalModelsPointNum += patch.getPointNum();	
	}

	float * positionsAndLabels = new float[totalModelsPointNum * 4];
	int index = 0;
	for (size_t i = 0; i < nboMatchingModels.size(); ++i)
	{
		for (int j = 0; j < nboMatchingModels[i].getPointNum(); ++j)
		{
			positionsAndLabels[4 * index + 0] = nboMatchingModels[i].getPointPositions()[j][0];
			positionsAndLabels[4 * index + 1] = nboMatchingModels[i].getPointPositions()[j][1];
			positionsAndLabels[4 * index + 2] = nboMatchingModels[i].getPointPositions()[j][2];
			positionsAndLabels[4 * index + 3] = i;
			++index;
		}
	}

	glGenBuffers(1, &matchedModelsVBO);
	glBindBuffer(GL_ARRAY_BUFFER, matchedModelsVBO);
	glBufferData(GL_ARRAY_BUFFER, totalModelsPointNum * sizeof(float) * 4, positionsAndLabels, GL_STREAM_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	delete [] positionsAndLabels;
	alreadyCopy = true;
}

void ScanningScheme::renderNBOMatchingModels(pangolin::OpenGlMatrix mvp)
{
	if (!alreadyCopy)
		copyNBOMatchingModelsIntoGL();

	drawProgram->Bind();

	drawProgram->setUniform(Uniform("MVP", mvp));
	drawProgram->setUniform(Uniform("modelNum", static_cast<int>(nboMatchingModels.size())));

	glBindBuffer(GL_ARRAY_BUFFER, matchedModelsVBO);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<GLvoid*>(0));

	glDrawArrays(GL_POINTS, 0, totalModelsPointNum);

	glDisableVertexAttribArray(0);

	drawProgram->Unbind();
}

void ScanningScheme::saveRecognizedObjects(const std::string & saveDir)
{
	for (auto & object : recognizedObjects)
	{
		object.second.save(saveDir + std::to_string(object.second.getLabelIndexes()[0]) + ".ply");
	}
}
