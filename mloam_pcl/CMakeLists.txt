cmake_minimum_required(VERSION 2.8.3)
project(mloam_pcl)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    CATKIN_DEPENDS
    #  DEPENDS system_lib
    INCLUDE_DIRS include
    LIBRARIES mloam_pcl
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(Eigen3 REQUIRED)


set(srcs
    src/voxel_grid_covariance_mloam.cpp
)

set(incs
    include/mloam_pcl/point_with_cov.hpp
    include/mloam_pcl/point_with_time.hpp
    include/mloam_pcl/voxel_grid_covariance_mloam.h
)

set(impl_incs
    include/mloam_pcl/voxel_grid_covariance_mloam_impl.hpp
)

set(LIB_NAME ${PROJECT_NAME})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
add_library(${LIB_NAME} ${srcs})
target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES} ${PCL_LIBS} ${Eigen3_LIBRARIES})
