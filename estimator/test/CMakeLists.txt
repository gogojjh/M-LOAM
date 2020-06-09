cmake_minimum_required(VERSION 2.8.3)
project(mloam_test)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake)

find_package(Eigen REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
# find_package(Ceres REQUIRED)
# find_package(Gflags REQUIRED)
# find_package(Glog REQUIRED)
# find_package(Sophus REQUIRED)
# find_package(libpointmatcher REQUIRED)
# find_package(G2O)

include_directories(${PCL_INCLUDE_DIRS} ${G2O_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
	3rdparty
	${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
	${CERES_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS}
	${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS}
)

# add_executable(test_ceres test_ceres.cpp)
# target_link_libraries(test_ceres ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test_pointiwithcov test_pointiwithcov.cpp)
# target_link_libraries(test_pointiwithcov ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test_merge_pointcloud_sr test_merge_pointcloud_sr.cpp)
# target_link_libraries(test_merge_pointcloud_sr ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test_merge_pointcloud_rhd test_merge_pointcloud_rhd.cpp)
# target_link_libraries(test_merge_pointcloud_rhd ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test_merge_pointcloud_rv test_merge_pointcloud_rv.cpp)
# target_link_libraries(test_merge_pointcloud_rv ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test_parse_bag test_parse_bag.cpp)
# target_link_libraries(test_parse_bag ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test_icp test_icp.cpp)
# target_link_libraries(test_icp ${libpointmatcher_LIBRARIES})

#add_executable(test_cuda_voxelgrid test_cuda_voxelgrid.cpp)
#target_link_libraries(test_cuda_voxelgrid ${PCL_LIBRARIES})



