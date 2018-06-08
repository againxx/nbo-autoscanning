cmake_minimum_required(VERSION 2.6.0)

project(libsanalysis)

find_package(PCL 1.3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin 0.1 REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})

link_directories(${PCL_LIB_DIRS})

add_definitions(${PCL_DEFINITIONS})

file(GLOB srcs *.cpp)
file(GLOB gco GCO/*.cpp)
file(GLOB cuda Cuda/*.cu)
file(GLOB utils_srcs Utils/*.cpp)

set(CUDA_ARCH_BIN "30 35 50 52 61" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for, Example: 1.0 1.2 or 10 12")

include(CudaComputeTargetFlags.cmake)
APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler;-fPIC")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-std=c++11;-O3;-I/usr/local/cudnn/v5.1/include;-L/usr/local/cuda/lib64;-L/usr/local/cudnn/v5.1/lib64;-lcudart;-lcublas;-lcudnn;-lcurand;-D_MWAITXINTRIN_H__INCLUDED")

CUDA_COMPILE(cuda_objs ${cuda})

#set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11 -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -g -Wall -std=c++11 -fopenmp")

#used for ycm
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(sanalysis SHARED
			${srcs}
			${utils_srcs}
			${gco}
			${cuda}
			${cuda_objs}
	)

target_link_libraries(sanalysis
					  ${Eigen_LIBRARIES}
					  ${CUDA_LIBRARIES}
					  ${PCL_LIBRARIES}
					  ${OpenCV_LIBS}
					  ${Pangolin_LIBRARIES}
					  "/usr/local/cudnn/v5.1/lib64/libcudnn.so"
					  "/usr/local/cuda/lib64/libcublas.so"
	)
