#ifndef CUDA_CUDAFUNCS_CUH_
#define CUDA_CUDAFUNCS_CUH_

#if __CUDA_ARCH__ < 300
#define MAX_THREADS 512
#else
#define MAX_THREADS 1024
#endif

#define CUDA_NUM_THREADS 512
#define CUDA_MAX_NUM_BLOCKS 2880

#include <Eigen/LU>
#include <chrono>

void computeTDFVoxelGrid(float * voxelGridOcc,
						 float * voxelGridTDF,
						 int voxelGridDimX,
						 int voxelGridDimY,
						 int voxelGridDimZ,
						 float voxelSize,
						 float truncMargin); 

void computeTDFVoxelGridSun(float * voxelGridOcc,
						 	float * voxelGridTDF,
						 	int voxelGridDimX,
						 	int voxelGridDimY,
						 	int voxelGridDimZ,
						 	float voxelSize,
						 	float truncMargin,
						 	int modelNum); 

void compute3DMatchDescriptor(int keypointNum,
							  int batchSize,
							  int descriptorSize,
							  float * keypointDescriptor,
							  float * keypointGrid,
							  float * voxelGridTDF,
							  int voxelGridDimX, int voxelGridDimY, int voxelGridDimZ);

void computeCorrespondPair(float * patchDescriptor,
						   float * modelDescriptor,
						   int modelNum,
						   int patchKeypointNum,
						   int modelKeypointNum,
						   int descriptorSize,
						   double paraMaxRank,
						   double paraCoefRank,
						   double paraExpRank,
						   double * correspondCost);

void labelPointToClosestSeed(int pointNum,
							 int keypointNum,
							 const float * pointPositions,
							 const int * seedIndexes,
							 int * pointLabels,
							 int * neighborIndexes);

#endif /* CUDA_CUDAFUNCS_CUH_ */
