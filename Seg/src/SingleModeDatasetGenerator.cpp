#include "SingleModeDatasetGenerator.h"

void SingleModeDatasetGenerator::generate(const std::string & pointCloudDir, const std::string & datasetDir)
{
	std::vector<std::string> modelNames;

	DirectoryReader dirReader(pointCloudDir);
	if (dirReader.isEmpty())
	{
		std::cerr << "Error: couldn't read point cloud directory!" << std::endl;
		return;
	}

	DescriptorGenerator descriptorGenerator;
	descriptorGenerator.setSamplingMode(DescriptorGenerator::UNIFORM_MODE);

	std::string fileName, filePath;

	// Compute keypoints and descriptors for point cloud files in dir
	while (dirReader.getNextFile(fileName, filePath))
	{
		FileNameParser fileNameParser(fileName);

		auto modelName = fileNameParser.getModelName();
		if (find(modelNames.cbegin(), modelNames.cend(), modelName) != modelNames.cend())
			continue;

		std::cout << "Current Model Name: " << modelName << std::endl;
		modelNames.push_back(modelName);

		descriptorGenerator.setPointData(filePath);
		KeypointRepresentation keyptRepr = descriptorGenerator.computeDescriptor(keypointNum);

		std::ofstream keypointOutputFile(datasetDir + "/" + modelName + "." + keypointFilePostfix);
		try
		{
			if (!keypointOutputFile)
			{
				throw std::runtime_error(datasetDir + "\nError: dataset directory is invalid!");
			}
			keypointOutputFile << keyptRepr;
			keypointOutputFile.close();
		}
		catch (std::runtime_error err)
		{
			std::cerr << err.what() << std::endl;
			keypointOutputFile.close();
			return;
		}

		//std::string localTDFDir = "/home/ustc-1314/ElasticFusion-master/Seg/LocalTDF/";
		//std::string savePath = localTDFDir + modelName;
		//if (access(savePath.c_str(), F_OK) == 0)
		//	descriptorGenerator.saveKeypointLocalTDFs(savePath);
		//else
		//{
		//	int createState = mkdir(savePath.c_str(), 0755);
		//	if (!createState)
		//	{
		//		std::cout << "Created directory: " << savePath << std::endl;
		//		descriptorGenerator.saveKeypointLocalTDFs(savePath);
		//	}
		//	else
		//	{
		//		std::cerr << "Error: could not create directory " << savePath << std::endl;
		//	}
		//}
	}
	std::cout << "Successfully generated " << modelNames.size() << " keypoint file(s)." << std::endl; 
}
