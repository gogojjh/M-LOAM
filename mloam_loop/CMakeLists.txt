cmake_minimum_required(VERSION 2.8.3)
project(mloam_loop)

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

	mloam_common
	mloam_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(
    include 
	${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS mloam_common mloam_msgs
  DEPENDS PCL
)

#add_executable(loop_fusion_node
#    src/pose_graph_node.cpp
#    src/pose_graph.cpp
#    src/keyframe.cpp
#    src/utility/CameraPoseVisualization.cpp
#    src/ThirdParty/DBoW/BowVector.cpp
#    src/ThirdParty/DBoW/FBrief.cpp
#    src/ThirdParty/DBoW/FeatureVector.cpp
#    src/ThirdParty/DBoW/QueryResults.cpp
#    src/ThirdParty/DBoW/ScoringObject.cpp
#    src/ThirdParty/DUtils/Random.cpp
#    src/ThirdParty/DUtils/Timestamp.cpp
#    src/ThirdParty/DVision/BRIEF.cpp
#    src/ThirdParty/VocabularyBinary.cpp
#    )
#
#target_link_libraries(loop_fusion_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 

#add_library(scan_context
#    src/scan_context.cpp
#)

#target_link_libraries(scan_context 
#    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} 
#    ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES}
#)

add_executable(loop_closure_node
	src/loop_closure_node.cpp
	src/parameters.cpp
	src/pose.cpp
	src/scan_context.cpp
	src/feature_extract.cpp
)

target_link_libraries(loop_closure_node
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} 
    ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES}
)

add_executable(test_scan_context
	src/test_scan_context.cpp
	src/parameters.cpp
	src/scan_context.cpp
)

target_link_libraries(test_scan_context
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} 
    ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES}
)
