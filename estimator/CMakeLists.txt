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
# find_package(glog REQUIRED)
find_package(Eigen3)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

catkin_package()

######################################################## float libraries
add_library(mloam_lib_1
    src/initial/initial_extrinsics.cpp
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/pose.cpp
    src/utility/visualization.cpp
    src/utility/cloud_visualizer.cpp
    src/factor/marginalization_factor.cpp
    src/lidarTracker/lidar_tracker.cpp
    )
target_link_libraries(mloam_lib_1 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} ${CERES_LIBRARIES})

######################################################## fix libraries
add_library(mloam_lib_2
    src/imageSegmenter/image_segmenter.cpp
    src/featureExtract/feature_extract.cpp
    src/factor/pose_local_parameterization.cpp
    src/utility/utility.cpp
    )
target_link_libraries(mloam_lib_2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} ${CERES_LIBRARIES})

add_executable(mloam_node src/rosNodeTest.cpp)
target_link_libraries(mloam_node mloam_lib_1 mloam_lib_2)

add_executable(lidar_mapper src/lidarMapper/lidar_mapper.cpp)
target_link_libraries(lidar_mapper mloam_lib_1 mloam_lib_2)

# add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
# target_link_libraries(kitti_gps_test mloam_lib_1 mloam_lib_2)

######## --------------------- TEST --------------------- ########
add_executable(test_ceres ../test/test_ceres.cpp)
target_link_libraries(test_ceres ${catkin_LIBRARIES} ${PCL_LIBS})

add_executable(test_merge_pointcloud ../test/test_merge_pointcloud.cpp)
target_link_libraries(test_merge_pointcloud ${catkin_LIBRARIES} ${PCL_LIBS})

#
