cmake_minimum_required(VERSION 2.8.3)

project(iroboscan)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
  nav_msgs
  geometry_msgs
  actionlib
  control_msgs
  gazebo_msgs
  tf
  move_base_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenNI2 REQUIRED)
find_package(efusion REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(sanalysis REQUIRED)
find_package(Threads REQUIRED)
find_package(darknet REQUIRED)

catkin_package(
	
)

include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${EFUSION_INCLUDE_DIR})
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${SANALYSIS_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${DARKNET_INCLUDE_DIRS})

file(GLOB srcs src/*.cpp)
file(GLOB tools_srcs src/Tools/*.cpp)

#set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1 -g -Wall -std=c++11")

add_definitions(-Dlinux=1)

#used for ycm
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(iroboscan
               ${srcs}
               ${tools_srcs}
)

target_link_libraries(iroboscan
                      ${ZLIB_LIBRARY}
                      ${Pangolin_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${EXTRA_LIBS}
                      ${EFUSION_LIBRARY}
                      ${OPENNI2_LIBRARY}
                      ${SUITESPARSE_LIBRARIES}
                      ${BLAS_LIBRARIES}
                      ${LAPACK_LIBRARIES}
					  ${catkin_LIBRARIES}
					  ${SANALYSIS_LIBRARIES}
					  ${DARKNET_LIBRARIES}
					  Threads::Threads
)
