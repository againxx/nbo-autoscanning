#include "cudafuncs.cuh"
#include "Marvin3DMatch.cuh"

// CUDA kernal function to compute TDF voxel grid values given a point cloud (warning: approximate, but fast)
__global__ void computeTDFKernel(int loopIDX, float * voxelGridOcc, float * voxelGridTDF,
								 int voxelGridDimX, int voxelGridDimY, int voxelGridDimZ,
								 float voxelSize, float truncMargin)
{
	int voxelIDX = loopIDX * CUDA_NUM_THREADS * CUDA_MAX_NUM_BLOCKS + blockIdx.x * CUDA_NUM_THREADS + threadIdx.x;
	if (voxelIDX >= (voxelGridDimX * voxelGridDimY * voxelGridDimZ))
		return;
	int pointGridZ = (int)floor((float)voxelIDX / ((float)voxelGridDimX * (float)voxelGridDimY));
	int pointGridY = (int)floor(((float)voxelIDX - ((float)pointGridZ * (float)voxelGridDimX * (float)voxelGridDimY)) / (float)voxelGridDimX);
	int pointGridX = (int)((float)voxelIDX - ((float)pointGridZ * (float)voxelGridDimX * (float)voxelGridDimY) - ((float)pointGridY * (float)voxelGridDimX));

	int searchRadius = (int)round(truncMargin / voxelSize);

	if (voxelGridOcc[voxelIDX] > 0)
	{
		voxelGridTDF[voxelIDX] = 1.0f;	// on surface
		return;
	}

	//Find closest surface point
	for (int i = max(0, pointGridX - searchRadius); i < min(voxelGridDimX, pointGridX + searchRadius + 1); ++i)
	{
		for (int j = max(0, pointGridY - searchRadius); j < min(voxelGridDimY, pointGridY + searchRadius + 1); ++j)
		{
			for (int k = max(0, pointGridZ - searchRadius); k < min(voxelGridDimZ, pointGridZ + searchRadius + 1); ++k)
			{
				int idx = k * voxelGridDimX * voxelGridDimY + j * voxelGridDimX + i;
				if (voxelGridOcc[idx] > 0)
				{
					float xd = (float)(pointGridX - i);
					float yd = (float)(pointGridY - j);
					float zd = (float)(pointGridZ - k);
					float dist = sqrtf(xd * xd + yd * yd + zd * zd) / (float)searchRadius;
					if ((1.0f - dist) > voxelGridTDF[voxelIDX])
						voxelGridTDF[voxelIDX] = 1.0f - dist;
				}
			}
		}
	}
}

__global__ void computeTDFKernelSun(int loopIDX, float * voxelGridOcc, float * voxelGridTDF,
								 int voxelGridDimX, int voxelGridDimY, int voxelGridDimZ,
								 float voxelSize, float truncMargin,int modelNum)
{
	int voxelIDX = loopIDX * CUDA_NUM_THREADS * CUDA_MAX_NUM_BLOCKS + blockIdx.x * CUDA_NUM_THREADS + threadIdx.x;
	if (voxelIDX >= (voxelGridDimX * voxelGridDimY * voxelGridDimZ))
		return;
	int pointGridZ = (int)floor((float)voxelIDX / ((float)voxelGridDimX * (float)voxelGridDimY));
	int pointGridY = (int)floor(((float)voxelIDX - ((float)pointGridZ * (float)voxelGridDimX * (float)voxelGridDimY)) / (float)voxelGridDimX);
	int pointGridX = (int)((float)voxelIDX - ((float)pointGridZ * (float)voxelGridDimX * (float)voxelGridDimY) - ((float)pointGridY * (float)voxelGridDimX));

	int searchRadius = (int)round(truncMargin / voxelSize);
	int voxelGirdSize = voxelGridDimX * voxelGridDimY * voxelGridDimZ;

	for (int l = 0; l < modelNum; ++l)
	{
		if (voxelGridOcc[voxelIDX + l * voxelGirdSize] > 0 && voxelGridOcc[voxelIDX + l * voxelGirdSize] < 1)
		{
			voxelGridTDF[voxelIDX + l * voxelGirdSize] = 1.0f;	// on surface
			continue;
		}

		//Find closest surface point
		for (int i = max(0, pointGridX - searchRadius); i < min(voxelGridDimX, pointGridX + searchRadius + 1); ++i)
		{
			for (int j = max(0, pointGridY - searchRadius); j < min(voxelGridDimY, pointGridY + searchRadius + 1); ++j)
			{
				for (int k = max(0, pointGridZ - searchRadius); k < min(voxelGridDimZ, pointGridZ + searchRadius + 1); ++k)
				{
					int idx = k * voxelGridDimX * voxelGridDimY + j * voxelGridDimX + i + l * voxelGirdSize;
					if (voxelGridOcc[idx] > 0 && voxelGridOcc[idx] < 1)
					{
						float xd = (float)(pointGridX - i);
						float yd = (float)(pointGridY - j);
						float zd = (float)(pointGridZ - k);
						float dist = sqrtf(xd * xd + yd * yd + zd * zd) / (float)searchRadius;
						if ((1.0f - dist) > voxelGridTDF[voxelIDX + l * voxelGirdSize])
							voxelGridTDF[voxelIDX + l * voxelGirdSize] = 1.0f - dist;
					}
				}
			}
		}
	}
}

void computeTDFVoxelGrid(float * voxelGridOcc,
						 float * voxelGridTDF,
						 int voxelGridDimX,
						 int voxelGridDimY,
						 int voxelGridDimZ,
						 float voxelSize,
						 float truncMargin)
{
	float * d_voxelGridOcc;
	float * d_voxelGridTDF;
	cudaMalloc(&d_voxelGridOcc, voxelGridDimX * voxelGridDimY * voxelGridDimZ * sizeof(float));
	cudaMalloc(&d_voxelGridTDF, voxelGridDimX * voxelGridDimY * voxelGridDimZ * sizeof(float));
	marvin::checkCUDA(__LINE__, cudaGetLastError());
	cudaMemcpy(d_voxelGridOcc, voxelGridOcc, voxelGridDimX * voxelGridDimY * voxelGridDimZ * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(d_voxelGridTDF, voxelGridTDF, voxelGridDimX * voxelGridDimY * voxelGridDimZ * sizeof(float), cudaMemcpyHostToDevice);
	marvin::checkCUDA(__LINE__, cudaGetLastError());

	int cudaLoopNum = (int)ceil((float)(voxelGridDimX * voxelGridDimY * voxelGridDimZ) / (float)(CUDA_NUM_THREADS * CUDA_MAX_NUM_BLOCKS));
	for (int i = 0; i < cudaLoopNum; ++i)
	{
		computeTDFKernel <<< CUDA_MAX_NUM_BLOCKS, CUDA_NUM_THREADS >>>(i, d_voxelGridOcc, d_voxelGridTDF,
				voxelGridDimX, voxelGridDimY, voxelGridDimZ,
				voxelSize, truncMargin);
	}

	cudaMemcpy(voxelGridTDF, d_voxelGridTDF, voxelGridDimX * voxelGridDimY * voxelGridDimZ *sizeof(float), cudaMemcpyDeviceToHost);
	marvin::checkCUDA(__LINE__, cudaGetLastError());

	cudaFree(d_voxelGridOcc);
	cudaFree(d_voxelGridTDF);
}

void computeTDFVoxelGridSun(float * voxelGridOcc,
						 float * voxelGridTDF,
						 int voxelGridDimX,
						 int voxelGridDimY,
						 int voxelGridDimZ,
						 float voxelSize,
						 float truncMargin,
						 int modelNum)
{
	float * d_voxelGridOcc;
	float * d_voxelGridTDF;
	cudaMalloc(&d_voxelGridOcc, modelNum * voxelGridDimX * voxelGridDimY * voxelGridDimZ * sizeof(float));
	cudaMalloc(&d_voxelGridTDF, modelNum * voxelGridDimX * voxelGridDimY * voxelGridDimZ * sizeof(float));
	marvin::checkCUDA(__LINE__, cudaGetLastError());
	cudaMemcpy(d_voxelGridOcc, voxelGridOcc, modelNum * voxelGridDimX * voxelGridDimY * voxelGridDimZ * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(d_voxelGridTDF, voxelGridTDF, modelNum * voxelGridDimX * voxelGridDimY * voxelGridDimZ * sizeof(float), cudaMemcpyHostToDevice);
	marvin::checkCUDA(__LINE__, cudaGetLastError());

	int cudaLoopNum = (int)ceil((float)(voxelGridDimX * voxelGridDimY * voxelGridDimZ) / (float)(CUDA_NUM_THREADS * CUDA_MAX_NUM_BLOCKS));
	for (int i = 0; i < cudaLoopNum; ++i)
	{
		computeTDFKernelSun <<< CUDA_MAX_NUM_BLOCKS, CUDA_NUM_THREADS >>>(i, d_voxelGridOcc, d_voxelGridTDF,
				voxelGridDimX, voxelGridDimY, voxelGridDimZ,
				voxelSize, truncMargin, modelNum);
	}

	cudaMemcpy(voxelGridTDF, d_voxelGridTDF, modelNum * voxelGridDimX * voxelGridDimY * voxelGridDimZ *sizeof(float), cudaMemcpyDeviceToHost);
	marvin::checkCUDA(__LINE__, cudaGetLastError());

	cudaFree(d_voxelGridOcc);
	cudaFree(d_voxelGridTDF);
}

void compute3DMatchDescriptor(int keypointNum,
							  int batchSize,
							  int descriptorSize,
							  float * keypointDescriptors,
							  float * keypointGrid,
							  float * voxelGridTDF,
							  int voxelGridDimX, int voxelGridDimY, int voxelGridDimZ)
{
	// Start Marvin network
	//std::string homePath(getenv("HOME"));
	//auto startTime = std::chrono::system_clock::now();
	//marvin::Net convnet(homePath + "/iroboscan/Seg/src/Cuda/3dmatch-net-test.json");
	//convnet.Malloc(marvin::Testing);
	//convnet.loadWeights(homePath + "/iroboscan/Seg/src/Cuda/3dmatch-weights-snapshot-137000.marvin");
	//auto endTime = std::chrono::system_clock::now();
	//auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	//std::cout << "Spend " << (double)duration.count() * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den  << " seconds loading weights." << std::endl;
	marvin::Response * rData;
	marvin::Response * rFeat;
	rData = Marvin3DMatch::getInstance().getConvolutionalNet()->getResponse("data");
	rFeat = Marvin3DMatch::getInstance().getConvolutionalNet()->getResponse("feat");
	//std::cout << "3D Match network architecture successfully loaded into Marvin!" << std::endl;

	// Run forward passes with Marvin to get 3DMatch descriptors for each keypoint
	StorageT * batchTDF = new StorageT[batchSize * 30 * 30 * 30];
	//std::cout << "Computing 3DMatch descriptor for " << keypointNum << " keypoints..." << std::endl;
	for (int i = 0; i < (keypointNum / batchSize); ++i)
	{
		for (int j = i * batchSize; j < (i + 1) * batchSize; ++j)
		{
			int batchKeyptIDX = j - i * batchSize;
			float keyptGridX = keypointGrid[3 * j];
			float keyptGridY = keypointGrid[3 * j + 1];
			float keyptGridZ = keypointGrid[3 * j + 2];

			StorageT * localVoxelGridTDF = new StorageT[30 * 30 * 30];
			int localVoxelIDX = 0;
			for (int z = keyptGridZ - 15; z < keyptGridZ + 15; ++z)
			{
				for (int y = keyptGridY - 15; y < keyptGridY + 15; ++y)
				{
					for (int x = keyptGridX - 15; x < keyptGridX + 15; ++x)
					{
						localVoxelGridTDF[localVoxelIDX] = CPUCompute2StorageT(voxelGridTDF[z * voxelGridDimX * voxelGridDimY + y * voxelGridDimX + x]);
						localVoxelIDX++;
					}
				}
			}
			memcpy(batchTDF + batchKeyptIDX * 30 * 30 * 30, localVoxelGridTDF, sizeof(StorageT) * 30 * 30 * 30);
			//for (int k = 0; k < 30 * 30 * 30; ++k)
			//{
			//	batchTDF[batchKeyptIDX * 30 * 30 * 30 + k] = localVoxelGridTDF[k];
			//}
			delete [] localVoxelGridTDF;
		}

		// Pass local TDF patches through Marvin
		cudaMemcpy(rData->dataGPU, batchTDF, rData->numBytes(), cudaMemcpyHostToDevice);
		marvin::checkCUDA(__LINE__, cudaGetLastError());
		Marvin3DMatch::getInstance().getConvolutionalNet()->forward();

		// Copy descriptor vectors from GPU to CPU memory
		StorageT * batchDescriptor = new StorageT[batchSize * descriptorSize];
		cudaMemcpy(batchDescriptor, rFeat->dataGPU, rFeat->numBytes(), cudaMemcpyDeviceToHost);
		marvin::checkCUDA(__LINE__, cudaGetLastError());

		for (int j = 0; j < batchSize * descriptorSize; ++j)
		{
			keypointDescriptors[i * batchSize * descriptorSize + j] = CPUStorage2ComputeT(batchDescriptor[j]);
		}
		delete [] batchDescriptor;
	}
	delete [] batchTDF;
}

__global__ void computeCorrespondPairKernel(float * patchDescriptor,
												float * modelDescriptor,
												int modelNum,
												int patchKeypointNum,
												int modelKeypointNum,
												int descriptorSize,
												double paraMaxRank,
												double paraCoefRank,
												double paraExpRank,
												double * correspondCost)
{
	int modelKeyptId = blockIdx.x;
	int patchKeyptId = threadIdx.x;

	__shared__ float descriptors[512];


//	__shared__ float descriptors[512];

		//descriptors[keyptIndex] = patchDescriptor[i * descriptorSize + keyptIndex];
		//if (keyptIndex < 12)
		//{
			//descriptors[keyptIndex + 500] = patchDescriptor[i * descriptorSize + keyptIndex + 500];
		//}
		//__syncthreads();
	
		//for (int j = 0; j < 20; ++j)
		//{
	if (modelKeyptId < modelKeypointNum)
	{
		for (int i = 0; i < modelNum; ++i)
		{
			descriptors[patchKeyptId] = modelDescriptor[i * modelKeypointNum * descriptorSize + modelKeyptId * descriptorSize + patchKeyptId];
			__syncthreads();

			if (patchKeyptId < patchKeypointNum)
			{
				double rankCost = 0.0;
				for (int j = 0; j < descriptorSize; ++j)
				{
					rankCost += pow(descriptors[j] - patchDescriptor[j * patchKeypointNum + patchKeyptId], 2);
				}
				rankCost = sqrt(rankCost);
				if (rankCost > paraMaxRank)
				{
					correspondCost[i * modelKeypointNum * patchKeypointNum + modelKeyptId * patchKeypointNum + patchKeyptId] = 999999;
				}
				else
				{
					correspondCost[i * modelKeypointNum * patchKeypointNum + modelKeyptId * patchKeypointNum + patchKeyptId] = paraCoefRank * pow(rankCost, paraExpRank);
				}
			}
			__syncthreads();
		}
	}
		//}
		//__syncthreads();
}

void computeCorrespondPair(float * patchDescriptor,
							   float * modelDescriptor,
							   int modelNum,
							   int patchKeypointNum,
							   int modelKeypointNum,
							   int descriptorSize,
							   double paraMaxRank,
							   double paraCoefRank,
							   double paraExpRank,
							   double * correspondCost)
{
	float * d_patchDescriptor;
	float * d_modelDescriptor;
	double * d_correspondCost;

	cudaMalloc(&d_patchDescriptor, patchKeypointNum * descriptorSize * sizeof(float));
	cudaMalloc(&d_modelDescriptor, modelNum * modelKeypointNum * descriptorSize * sizeof(float));
	cudaMalloc(&d_correspondCost, modelNum * patchKeypointNum * modelKeypointNum * sizeof(double));

	float * patchDescriptorTrans = new float[patchKeypointNum * descriptorSize];
	for (int i = 0; i < patchKeypointNum; ++i)
	{
		for (int j = 0; j < descriptorSize; ++j)
		{
			patchDescriptorTrans[j * patchKeypointNum + i] = patchDescriptor[i * descriptorSize + j];
		}
	}

	cudaMemcpy(d_patchDescriptor, patchDescriptorTrans, patchKeypointNum * descriptorSize * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(d_modelDescriptor, modelDescriptor, modelNum * modelKeypointNum * descriptorSize * sizeof(float), cudaMemcpyHostToDevice);

	int threadsPerBlock = 512;
	int blocksPerGrid = 512;


	//computeCorrespondPairKernel <<< blocksPerGrid, threadsPerBlock, 20 * descriptorSize * sizeof(float)>>>(d_patchDescriptor, d_modelDescriptor, 
			//patchKeypointNum, modelKeypointNum, descriptorSize, paraMaxRank, paraCoefRank, paraExpRank, d_correspondCost);
	cudaEvent_t start1;
	cudaEventCreate(&start1);
	cudaEvent_t stop1;
	cudaEventCreate(&stop1);
	cudaEventRecord(start1, NULL);
	computeCorrespondPairKernel <<< blocksPerGrid, threadsPerBlock>>>(d_patchDescriptor, d_modelDescriptor, modelNum, 
			patchKeypointNum, modelKeypointNum, descriptorSize, paraMaxRank, paraCoefRank, paraExpRank, d_correspondCost);
	cudaEventRecord(stop1, NULL);
	cudaEventSynchronize(stop1);
	float mescTotal1 = 0.0f;
	cudaEventElapsedTime(&mescTotal1, start1, stop1);
	std::cout << mescTotal1 << " msec" << std::endl;

	cudaMemcpy(correspondCost, d_correspondCost, modelNum * patchKeypointNum * modelKeypointNum * sizeof(double), cudaMemcpyDeviceToHost);

	cudaFree(d_patchDescriptor);
	cudaFree(d_modelDescriptor);
	cudaFree(d_correspondCost);
	delete [] patchDescriptorTrans;
}

__global__ void labelPointToClosestSeedKernel(int loopIndex, int pointNum, int keypointNum, float * pointPositions, int * seedIndexes, int * pointLabels, int * neighborIndexes)
{
	int pointIndex = loopIndex * CUDA_NUM_THREADS * CUDA_MAX_NUM_BLOCKS + blockIdx.x * CUDA_NUM_THREADS + threadIdx.x;

	if (pointIndex >= pointNum)
		return;

	float posX = pointPositions[3 * pointIndex];
	float posY = pointPositions[3 * pointIndex + 1];
	float posZ = pointPositions[3 * pointIndex + 2];

	int closestSeedIndex = -1;
	int neighborIndex = -1;
	float minDistance = 99999;
	float neighborDistance = 99999;
	for (int i = 0; i < keypointNum; ++i)
	{
		int seedIndex = seedIndexes[i];
		float distance = sqrt(pow(posX - pointPositions[3 * seedIndex], 2) + pow(posY - pointPositions[3 * seedIndex + 1], 2) + pow(posZ - pointPositions[3 * seedIndex + 2], 2));
		if (distance < minDistance)
		{
			neighborIndex = closestSeedIndex;
			neighborDistance = minDistance;
			closestSeedIndex = i;
			minDistance = distance;
		}
		else if (distance < neighborDistance)
		{
			neighborIndex = i;
			neighborDistance = distance;
		}
	}
	pointLabels[pointIndex] = closestSeedIndex;
	neighborIndexes[pointIndex] = neighborIndex;
}

void labelPointToClosestSeed(int pointNum,
							 int keypointNum,
							 const float * pointPositions,
							 const int * seedIndexes,
							 int * pointLabels,
							 int * neighborIndexes)
{
	float * d_pointPositions;
	int * d_seedIndexes;
	int * d_pointLabels;
	int * d_neighborIndexes;

	cudaMalloc(&d_pointPositions, pointNum * 3 * sizeof(float));
	cudaMalloc(&d_seedIndexes, keypointNum * sizeof(int));
	cudaMalloc(&d_pointLabels, pointNum * sizeof(int));
	cudaMalloc(&d_neighborIndexes, pointNum * sizeof(int));

	cudaMemcpy(d_pointPositions, pointPositions, pointNum * 3 * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(d_seedIndexes, seedIndexes, keypointNum * sizeof(int), cudaMemcpyHostToDevice);

	int cudaLoopNum = (int)ceil((float)(pointNum) / (float)(CUDA_NUM_THREADS * CUDA_MAX_NUM_BLOCKS));
	for (int i = 0; i < cudaLoopNum; ++i)
	{
		labelPointToClosestSeedKernel <<< CUDA_MAX_NUM_BLOCKS, CUDA_NUM_THREADS >>>(i, pointNum, keypointNum, d_pointPositions, d_seedIndexes, d_pointLabels, d_neighborIndexes);
	}

	cudaMemcpy(pointLabels, d_pointLabels, pointNum * sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(neighborIndexes, d_neighborIndexes, pointNum * sizeof(int), cudaMemcpyDeviceToHost);

	cudaFree(d_pointPositions);
	cudaFree(d_seedIndexes);
	cudaFree(d_pointLabels);
	cudaFree(d_neighborIndexes);
}
