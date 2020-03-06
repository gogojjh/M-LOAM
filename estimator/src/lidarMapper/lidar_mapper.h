// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <map>
#include <cassert>
#include <algorithm>
#include <utility>

#include "common/common.hpp"
#include "common/types/type.h"
#include "common/publisher.hpp"

#include "mloam_msgs/Extrinsics.h"

#include "mloam_pcl/point_with_cov.hpp"
#include "mloam_pcl/voxel_grid_covariance_mloam.h"
#include "mloam_pcl/voxel_grid_covariance_mloam_impl.hpp"

#include "../utility/tic_toc.h"
#include "../utility/utility.h"
#include "../estimator/pose.h"
#include "../estimator/parameters.h"
#include "../featureExtract/feature_extract.h"
#include "../factor/lidar_map_plane_norm_factor.hpp"
#include "../factor/lidar_edge_factor.hpp"
#include "../factor/lidar_plane_norm_factor.hpp"
#include "../factor/pose_local_parameterization.h"

void evalDegenracy(PoseLocalParameterization *local_parameterization, const ceres::CRSMatrix &jaco);

// pointToFS turns a 4x1 homogeneous point into a special 4x6 matrix
Eigen::Matrix<double, 4, 6> pointToFS(const Eigen::Vector4d &point)
{
    Eigen::Matrix<double, 4, 6> G = Eigen::Matrix<double, 4, 6>::Zero();
    G.block<3, 3>(0, 0) = point(3) * Eigen::Matrix3d::Identity();
    G.block<3, 3>(0, 3) = -Utility::skewSymmetric(point.block<3, 1>(0, 0));
    return G;
}

void evalPointUncertainty(const common::PointI &pi, common::PointIWithCov &po, const Eigen::Matrix<double, 6, 6> &cov_pose)
{
    //THETA: diag(P, Phi, Z) includes the translation, rotation, measurement uncertainty
    Eigen::Matrix<double, 9, 9> cov_input = Eigen::Matrix<double, 9, 9>::Zero();
    cov_input.topLeftCorner<6, 6>() = cov_pose;
    cov_input.bottomRightCorner<3, 3>() = COV_MEASUREMENT;

    Eigen::Vector4d point_curr(pi.x, pi.y, pi.z, 1);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 4, 3> D;
    D << 1, 0, 0,
         0, 1, 0,
         0, 0, 1,
         0, 0, 0;
    Eigen::Matrix<double, 4, 9> G = Eigen::Matrix<double, 4, 9>::Zero();
    G.block<4, 6>(0, 0) = pointToFS(T * point_curr);
    G.block<4, 3>(0, 6) = T * D;
    Eigen::Matrix3d cov_point = Eigen::Matrix4d(G * cov_input * G.transpose()).topLeftCorner<3, 3>(); // 3x3
    common::appendCov(pi, po, cov_point);

    // std::cout << cov_input << std::endl;
    // std::cout << G << std::endl;
    // std::cout << "evalUncertainty:" << std::endl
    //           << point_curr.transpose() << std::endl
    //           << cov_point << std::endl;
    // exit(EXIT_FAILURE);
}

//
