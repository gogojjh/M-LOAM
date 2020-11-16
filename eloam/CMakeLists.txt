cmake_minimum_required(VERSION 2.8.3)
project(eloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -fPIC")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS} 
	${CERES_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

cs_add_executable(eloam_node
	src/event_frame.cpp
	src/eloam_node.cpp
)
target_link_libraries(eloam_node
    ${catkin_LIBRARIES} 
	${OpenCV_LIBS} 
	${PCL_LIBRARIES} 
	${CERES_LIBRARIES}
)

########################################### TEST ################
# cs_add_executable(test_generate_data test/test_generate_data.cpp)
# target_link_libraries(test_generate_data ${PCL_LIBRARIES})

cs_install()
