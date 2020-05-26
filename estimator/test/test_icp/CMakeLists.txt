cmake_minimum_required(VERSION 2.8.3)
project(test_icp)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

find_package(libpointmatcher REQUIRED)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(${EIGEN3_INCLUDE_DIR} 
                    ${libpointmatcher_INCLUDE_DIRS})

add_executable(test_icp test_icp.cpp)
target_link_libraries(test_icp ${libpointmatcher_LIBRARIES})
#
