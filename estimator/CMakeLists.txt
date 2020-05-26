cmake_minimum_required(VERSION 2.8.3)
project(mloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
#add_compile_options(-std=c++17)
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
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
# find_package(Sophus REQUIRED)
# find_package(libpointmatcher REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
        3rdparty
        ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS mloam_msgs mloam_pcl
  DEPENDS PCL
)

######################################################## float libraries
add_library(mloam_lib_1
    src/estimator/estimator.cpp
    src/featureExtract/feature_extract.cpp
    src/lidarTracker/lidar_tracker.cpp
    src/utility/visualization.cpp
    src/utility/utility.cpp
    )
target_link_libraries(mloam_lib_1 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} ${CERES_LIBRARIES})

######################################################## fix libraries
add_library(mloam_lib_2
    src/estimator/parameters.cpp
    src/estimator/pose.cpp
    src/initial/initial_extrinsics.cpp
    src/imageSegmenter/image_segmenter.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/marginalization_factor.cpp
    src/utility/cloud_visualizer.cpp
    )
target_link_libraries(mloam_lib_2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} ${CERES_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})

######## --------------------- EXECUTABLE --------------------- ########
add_executable(mloam_node_sr src/rosNodeSR.cpp)
target_link_libraries(mloam_node_sr mloam_lib_1 mloam_lib_2)

add_executable(mloam_node_rhd src/rosNodeRHD.cpp)
target_link_libraries(mloam_node_rhd mloam_lib_1 mloam_lib_2)

add_executable(mloam_node_rv src/rosNodeRV.cpp)
target_link_libraries(mloam_node_rv mloam_lib_1 mloam_lib_2)

add_executable(lidar_mapper src/lidarMapper/lidar_mapper.cpp)
target_link_libraries(lidar_mapper mloam_lib_1 mloam_lib_2)

######## --------------------- TEST --------------------- ########
# add_executable(test_ceres ../test/test_ceres.cpp)
# target_link_libraries(test_ceres ${catkin_LIBRARIES} ${PCL_LIBS})

# add_executable(test_pointiwithcov ../test/test_pointiwithcov.cpp)
# target_link_libraries(test_pointiwithcov ${catkin_LIBRARIES} ${PCL_LIBS})

# add_executable(test_merge_pointcloud_sr ../test/test_merge_pointcloud_sr.cpp)
# target_link_libraries(test_merge_pointcloud_sr ${catkin_LIBRARIES} ${PCL_LIBS})

# add_executable(test_merge_pointcloud_rhd ../test/test_merge_pointcloud_rhd.cpp)
# target_link_libraries(test_merge_pointcloud_rhd ${catkin_LIBRARIES} ${PCL_LIBS})

# add_executable(test_merge_pointcloud_rv ../test/test_merge_pointcloud_rv.cpp)
# target_link_libraries(test_merge_pointcloud_rv ${catkin_LIBRARIES} ${PCL_LIBS})

# add_executable(test_parse_bag ../test/test_parse_bag.cpp)
# target_link_libraries(test_parse_bag ${catkin_LIBRARIES} ${PCL_LIBS})

# add_executable(test_icp ../test/test_icp.cpp)
# target_link_libraries(test_icp ${libpointmatcher_LIBRARIES})
#
