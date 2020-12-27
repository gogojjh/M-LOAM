cmake_minimum_required(VERSION 2.8.3)
project(mloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
	tf
	roscpp
	std_msgs
	nav_msgs
    sensor_msgs
	geometry_msgs
	cv_bridge
	image_transport
	message_generation

	pcl_ros
	pcl_conversions

	mloam_common
	mloam_msgs
	mloam_pcl
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
	3rdparty
	${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
#   INCLUDE_DIRS include
  CATKIN_DEPENDS mloam_common mloam_msgs mloam_pcl
  DEPENDS PCL
)

######################################################## float libraries
add_library(mloam_lib
    src/initial/initial_extrinsics.cpp
    src/featureExtract/feature_extract.cpp
    src/imageSegmenter/image_segmenter.cpp
    src/lidarTracker/lidar_tracker.cpp
    src/estimator/parameters.cpp
    src/estimator/pose.cpp
    src/estimator/estimator.cpp
    src/utility/utility.cpp
    src/utility/cloud_visualizer.cpp
    src/utility/visualization.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/marginalization_factor.cpp
)
target_link_libraries(mloam_lib
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} 
    ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES}
)

######## --------------------- EXECUTABLE --------------------- ########
add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(mloam_node_sr src/rosNodeSR.cpp)
target_link_libraries(mloam_node_sr mloam_lib)

add_executable(mloam_node_rhd src/rosNodeRHD.cpp)
target_link_libraries(mloam_node_rhd mloam_lib)

add_executable(mloam_node_rv_hercules src/rosNodeRVHercules.cpp)
target_link_libraries(mloam_node_rv_hercules mloam_lib)

add_executable(mloam_node_rv_oxford src/rosNodeRVOxford.cpp)
target_link_libraries(mloam_node_rv_oxford mloam_lib)

add_executable(mloam_node_rv_kitti src/rosNodeRVKITTI.cpp)
target_link_libraries(mloam_node_rv_kitti mloam_lib)

add_executable(lidar_mapper_keyframe src/lidarMapper/lidar_mapper_keyframe.cpp)
target_link_libraries(lidar_mapper_keyframe mloam_lib)

