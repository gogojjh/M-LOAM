cmake_minimum_required(VERSION 2.8.3)
project(mloam_common)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
#add_compile_options(-std=c++14)
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    # ROS core
    roscpp
    # ROS messages
    std_msgs
    sensor_msgs
    # ROS PCL
    pcl_conversions
    pcl_ros
    # ROS tf
    tf
    tf_conversions
    # 3rd modules for ROS, like Eigen
    cmake_modules
)

## System dependencies are found with CMake's conventions
### PCL
find_package(PCL REQUIRED COMPONENTS
    common
    io
    filter
    surface
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
### Eigen
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS roscpp tf tf_conversions
    #  DEPENDS system_lib
    INCLUDE_DIRS include
    LIBRARIES mloam_common
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_library(${PROJECT_NAME}
    src/algos/hungarian_bigraph_matcher.cpp
    src/timing.cpp
)

## http://mariobadr.com/creating-a-header-only-library-with-cmake.html
#add_library(${PROJECT_NAME} INTERFACE)
#target_include_directories(${PROJECT_NAME} INTERFACE include/common)

## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_node
#    ${catkin_LIBRARIES}
#    ${PCL_LIBRARIES}
#    ${Eigen3_LIBRARIES}
#)

#############
## Install ##
#############

# ...
