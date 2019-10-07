cmake_minimum_required(VERSION 2.8.3)
project(mloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport

    pcl_ros
    pcl_conversions

    mloam_common
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
find_package(Eigen3)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

catkin_package()

add_library(mloam_lib
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/pose.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/cloud_visualizer.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/initial/initial_extrinsics.cpp
    src/featureTracker/feature_tracker.cpp
    src/featureExtract/feature_extract.cpp
    src/lidarTracker/lidar_tracker.cpp
    )
target_link_libraries(mloam_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} ${CERES_LIBRARIES})

add_executable(mloam_node src/rosNodeTest.cpp)
target_link_libraries(mloam_node mloam_lib)

add_executable(lidar_mapper src/lidarMapper/lidar_mapper.cpp)
target_link_libraries(lidar_mapper mloam_lib)

# add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
# target_link_libraries(kitti_gps_test mloam_lib)
