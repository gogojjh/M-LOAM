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

/*
 * pi: the original point for evaluating uncertainty
 * cov_point: associated covariance of the point
 * pose: pose
 * cov_pose: associated covariance of the pose
 */
template<typename PointType>
void evalPointUncertainty(const PointType &pi, Eigen::Matrix3d &cov_point, const Pose &pose, const Eigen::Matrix<double, 6, 6> &cov_pose)
{
    //THETA: diag(P, Phi, Z) includes the translation, rotation, measurement uncertainty
    Eigen::Matrix<double, 9, 9> cov_input = Eigen::Matrix<double, 9, 9>::Zero();
    cov_input.topLeftCorner<6, 6>() = cov_pose;
    cov_input.bottomRightCorner<3, 3>() = COV_MEASUREMENT;

    Eigen::Vector4d point_curr(pi.x, pi.y, pi.z, 1);
    Eigen::Matrix4d T = pose.T_;
    // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 4, 3> D;
    D << 1, 0, 0,
         0, 1, 0,
         0, 0, 1,
         0, 0, 0;
    Eigen::Matrix<double, 4, 9> G = Eigen::Matrix<double, 4, 9>::Zero();
    G.block<4, 6>(0, 0) = pointToFS(T * point_curr);
    G.block<4, 3>(0, 6) = T * D;
    cov_point = Eigen::Matrix4d(G * cov_input * G.transpose()).topLeftCorner<3, 3>(); // 3x3
//     std::cout << cov_input << std::endl;
//     std::cout << G << std::endl;
//     std::cout << "evalUncertainty:" << std::endl
//               << point_curr.transpose() << std::endl
//               << cov_point << std::endl;
//     exit(EXIT_FAILURE);
}

Eigen::Matrix<double, 6, 6> adjointMatrix(const Eigen::Matrix4d &T)
{
    Eigen::Matrix<double, 6, 6> AdT = Eigen::Matrix<double, 6, 6>::Zero();
    AdT.topLeftCorner<3, 3>() = T.topLeftCorner<3 ,3>();
    AdT.topRightCorner<3, 3>() = Utility::skewSymmetric(T.topRightCorner<3, 1>()) * T.topLeftCorner<3, 3>();
    AdT.bottomRightCorner<3, 3>() = T.topLeftCorner<3, 3>();
    return AdT;
}

Eigen::Matrix3d covop1(const Eigen::Matrix3d &B)
{
    Eigen::Matrix3d A = -B.trace() * Eigen::Matrix3d::Identity() + B;
    return A;
}

Eigen::Matrix3d covop2(const Eigen::Matrix3d &B, const Eigen::Matrix3d &C)
{
    Eigen::Matrix3d A = covop1(B) * covop1(C) * covop1(C * B);
    return A;
}

// fixed: topLeftCorner<3, 3>()
// dynamic: topLeftCorner(3, 3)
void compoundPoseWithCov(const Pose &pose_1, const Eigen::Matrix<double, 6, 6> &cov_1,
                        const Pose &pose_2, const Eigen::Matrix<double, 6, 6> &cov_2,
                        Pose &pose_cp, Eigen::Matrix<double, 6, 6> &cov_cp, const int method = 1)
{  
    pose_cp = Pose(pose_1.T_ * pose_2.T_);
    Eigen::Matrix<double, 6, 6> AdT1 = adjointMatrix(pose_1.T_); // the adjoint matrix of T1
    Eigen::Matrix<double, 6, 6> cov_2_prime = AdT1 * cov_2 * AdT1.transpose();
    if (method == 1)
    {
        cov_cp = cov_1 + cov_2_prime;
    } else
    if (method == 2)
    {
        Eigen::Matrix3d cov_1_rr = cov_1.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_1_rp = cov_1.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_1_pp = cov_1.bottomRightCorner<3, 3>();

        Eigen::Matrix3d cov_2_rr = cov_2_prime.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_2_rp = cov_2_prime.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_2_pp = cov_2_prime.bottomRightCorner<3, 3>();

        Eigen::Matrix<double, 6, 6> A1 = Eigen::Matrix<double, 6, 6>::Zero();
        A1.topLeftCorner<3, 3>() = covop1(cov_1_pp);
        A1.topRightCorner<3, 3>() = covop1(cov_1_rp + cov_1_rp.transpose());
        A1.bottomRightCorner<3, 3>() = covop1(cov_1_pp);

        Eigen::Matrix<double, 6, 6> A2 = Eigen::Matrix<double, 6, 6>::Zero();
        A2.topLeftCorner<3, 3>() = covop1(cov_2_pp);
        A2.topRightCorner<3, 3>() = covop1(cov_2_rp + cov_2_rp.transpose());
        A2.bottomRightCorner<3, 3>() = covop1(cov_2_pp);

        Eigen::Matrix3d Brr = covop2(cov_1_pp, cov_2_rr) + covop2(cov_1_rp.transpose(), cov_2_rp) 
                            + covop2(cov_1_rp, cov_2_rp.transpose()) + covop2(cov_1_rr, cov_2_pp);
        Eigen::Matrix3d Brp = covop2(cov_1_pp, cov_2_rp.transpose()) + covop2(cov_1_rp.transpose(), cov_2_pp);
        Eigen::Matrix3d Bpp = covop2(cov_1_pp, cov_2_pp);
        Eigen::Matrix<double, 6, 6> B = Eigen::Matrix<double, 6, 6>::Zero();
        B.topLeftCorner<3, 3>() = Brr;
        B.topRightCorner<3, 3>() = Brp;
        B.bottomLeftCorner<3, 3>() = Brp.transpose();
        B.bottomRightCorner<3, 3>() = Bpp;
        
        cov_cp = cov_1 + cov_2_prime 
                + (A1 * cov_2_prime + cov_2_prime * A1.transpose() + A2 * cov_1 + cov_1 * A2.transpose()) / 12 
                + B / 4;        
    } else
    {
        printf("[compoundPoseWithCov] No %dth method !\n", method);
        cov_cp.setZero();
    }

 }


//
