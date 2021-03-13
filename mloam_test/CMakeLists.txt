cmake_minimum_required(VERSION 2.8.3)
project(mloam_test)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	tf
	cv_bridge
	image_transport
	message_generation
	visualization_msgs

	pcl_ros
	pcl_conversions

	mloam_common
	mloam_msgs
	mloam_pcl
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(G2O)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
	3rdparty
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
	${CERES_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS}
	${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS}
)

catkin_package(
#   INCLUDE_DIRS include
  CATKIN_DEPENDS mloam_msgs mloam_pcl
  DEPENDS PCL
)

# add_executable(test_ceres test_ceres.cpp)
# target_link_libraries(test_ceres ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(test_pointiwithcov src/test_pointiwithcov.cpp)
#target_link_libraries(test_pointiwithcov ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(test_merge_pointcloud_sr src/test_merge_pointcloud_sr.cpp)
#target_link_libraries(test_merge_pointcloud_sr ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(test_merge_pointcloud_rhd src/test_merge_pointcloud_rhd.cpp)
#target_link_libraries(test_merge_pointcloud_rhd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(test_merge_pointcloud_rv_hercules src/test_merge_pointcloud_rv_hercules.cpp)
#target_link_libraries(test_merge_pointcloud_rv_hercules ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(test_generate_bag_from_data_hercules src/test_generate_bag_from_data_hercules.cpp)
#target_link_libraries(test_generate_bag_from_data_hercules ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(test_generate_noisy_bag src/test_generate_noisy_bag.cpp)
#target_link_libraries(test_generate_noisy_bag ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test_parse_bag test_parse_bag.cpp)
# target_link_libraries(test_parse_bag ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(test_icp test_icp/test_icp.cpp)
# target_link_libraries(test_icp ${libpointmatcher_LIBRARIES})

# add_executable(test_cuda_voxelgrid test_cuda_voxelgrid.cpp)
# target_link_libraries(test_cuda_voxelgrid ${PCL_LIBRARIES})

add_library(poseIO SHARED src/g2oIO/PoseGraphIO.cpp)
target_link_libraries(poseIO g2o_core g2o_stuff g2o_types_slam3d)

# add_executable(test_generate_full_map test_generate_full_map.cpp)
# target_link_libraries(test_generate_full_map poseIO mloam_lib_1)

add_executable(test_merge_pose_map src/test_merge_pose_map.cpp)
target_link_libraries(test_merge_pose_map poseIO ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  
					  ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(test_rnd src/test_rnd.cpp)
#target_link_libraries(test_rnd)

#add_executable(test_rgi src/test_rgi.cpp)
#target_link_libraries(test_rgi ${catkin_LIBRARIES})

#add_executable(test_generate_bag_from_data_oxford 
#	src/test_generate_bag_from_data_oxford.cpp ../estimator/src/estimator/pose.cpp)
#target_link_libraries(test_generate_bag_from_data_oxford
#    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} 
#    ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
#    ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES}
#)

#add_executable(test_merge_pointcloud_oxford 
#	src/test_merge_pointcloud_oxford.cpp ../estimator/src/estimator/pose.cpp)
#target_link_libraries(test_merge_pointcloud_oxford
#    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} 
#    ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
#    ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES}
#)

#add_executable(test_jacobian_memory 
#	src/test_jacobian_memory.cpp ../estimator/src/estimator/pose.cpp)
#target_link_libraries(test_jacobian_memory
#    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} 
#    ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
#    ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES}
#)

# add_executable(ceres_robust_curve_fitting 
#     src/ceres_robust_curve_fitting.cpp src/impl_loss_function.cpp)
# target_link_libraries(ceres_robust_curve_fitting ${catkin_LIBRARIES} 
# 	${CERES_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(ceres_robust_curve_fitting_2 
#    src/ceres_robust_curve_fitting_2.cpp src/impl_loss_function.cpp)
#target_link_libraries(ceres_robust_curve_fitting_2 ${catkin_LIBRARIES} 
#	${CERES_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(ceres_fitting_uct src/ceres_fitting_uct.cpp)
#target_link_libraries(ceres_fitting_uct ${catkin_LIBRARIES} 
#	${CERES_LIBRARIES} ${GLOG_LIBRARIES})

# add_executable(ceres_simple_bundle_adjustment
#     src/ceres_simple_bundle_adjustment.cpp src/impl_loss_function.cpp)
# target_link_libraries(ceres_simple_bundle_adjustment ${catkin_LIBRARIES} 
# 	${CERES_LIBRARIES} ${GLOG_LIBRARIES})

#add_executable(test_timing src/test_timing.cpp)
#target_link_libraries(test_timing ${catkin_LIBRARIES})

#add_executable(test_quaternion src/test_quaternion.cpp)
#target_link_libraries(test_quaternion ${catkin_LIBRARIES})
