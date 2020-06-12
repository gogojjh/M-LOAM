/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of M-LOAM (https://ram-lab.com/file/jjiao/m-loam).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Jianhao JIAO (jiaojh1994@gmail.com)
 *******************************************************/

#pragma once

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <math.h>
#include <vector>
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
#include <omp.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>

#include "common/common.hpp"
#include "common/types/type.h"
#include "common/publisher.hpp"
#include "common/color.hpp"

#include "mloam_msgs/Extrinsics.h"

#include "mloam_pcl/point_with_cov.hpp"
#include "mloam_pcl/voxel_grid_covariance_mloam.h"
#include "mloam_pcl/voxel_grid_covariance_mloam_impl.hpp"

#include "../save_statistics.hpp"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"
#include "../estimator/pose.h"
#include "../estimator/parameters.h"
#include "../featureExtract/feature_extract.h"
#include "../factor/lidar_map_plane_norm_factor.hpp"
#include "../factor/lidar_plane_norm_factor.hpp"
#include "../factor/pose_local_parameterization.h"

#define MAX_FEATURE_SELECT_TIME 10 // 10ms
#define MAX_RANDOM_QUEUE_TIME 20

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");
DEFINE_bool(with_ua, true, "with or without the awareness of uncertainty");
DEFINE_double(gf_ratio, 0.2, "with or without the good features selection");

void evalHessian(const ceres::CRSMatrix &jaco, Eigen::Matrix<double, 6, 6> &mat_H);

void evalDegenracy(const Eigen::Matrix<double, 6, 6> &mat_H, PoseLocalParameterization *local_parameterization);

// ***************************************************************** Barfoot's method on associating uncertainty on SE3

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
    // THETA: diag(P, Phi, Z) includes the translation, rotation, measurement uncertainty
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
    // std::cout << cov_input << std::endl;
    // std::cout << G << std::endl;
    // std::cout << "evalUncertainty:" << std::endl
    //           << point_curr.transpose() << std::endl
    //           << cov_point << std::endl;
    // exit(EXIT_FAILURE);
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
                        Pose &pose_cp, Eigen::Matrix<double, 6, 6> &cov_cp, const int method = 2)
{  
    pose_cp.q_ = pose_1.q_ * pose_2.q_;
    pose_cp.t_ = pose_1.q_ * pose_2.t_ + pose_1.t_;
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
    // std::cout << pose_1 << std::endl << cov_1 << std::endl;
    // std::cout << pose_2 << std::endl << cov_2 << std::endl;
    // std::cout << pose_cp << std::endl << cov_cp << std::endl;
    // exit(EXIT_FAILURE);
}

void evaluateFeatJacobian(const double *para_pose,
                          const PointPlaneFeature &feature,
                          const Eigen::Matrix3d &cov_matrix,
                          Eigen::MatrixXd &mat_jaco)
{
    LidarMapPlaneNormFactor *f = new LidarMapPlaneNormFactor(feature.point_, feature.coeffs_, cov_matrix);
    const double **param = new const double *[1];
    param[0] = para_pose;
    double *res = new double[1];
    double **jaco = new double *[1];
    jaco[0] = new double[3 * 7];
    f->Evaluate(param, res, jaco);
    Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > mat_jacobian(jaco[0]);
    mat_jaco = mat_jacobian.topLeftCorner<3, 6>();
}

void goodFeatureSelect(const double *para_pose,
                       const std::vector<PointICovCloud> laser_cloud_surf_cov,
                       const std::vector<PointICovCloud> laser_cloud_corner_cov,
                       const std::vector<PointPlaneFeature> &all_features,
                       const size_t &num_all_features,
                       std::vector<size_t> &sel_feature_idx,
                       const float &gf_ratio = 0.2)
{
    // create a query of the index of the valid features
    std::vector<size_t> all_feature_idx(num_all_features);
    std::vector<int> feature_visited(num_all_features, -1);
    std::iota(all_feature_idx.begin(), all_feature_idx.end(), 0);
  
    size_t num_use_features;
    if (gf_ratio >= 1.0)
    {
        sel_feature_idx = all_feature_idx;
        printf("use all features: %lu!\n", sel_feature_idx.size());
        return;
    } 
    else if (gf_ratio <= 0.0)
    {
        num_use_features = 100;
    }
    else
    {
        num_use_features = static_cast<size_t>(num_all_features * gf_ratio);
    }
    // printf("size of good features: %lu\n", num_use_features);

    // size of the random subset
    size_t size_rnd_subset = static_cast<size_t>(float(num_all_features) / float(num_use_features) * 1.0);
    // size_t size_rnd_subset = static_cast<size_t>(float(num_all_features) / float(num_use_features) * 2.3);
    LOG_EVERY_N(INFO, 10) << "[goodFeatureSelct] size of matrix subset: " << size_rnd_subset;

    // the most informative Hessian matrix
    Eigen::Matrix<double, 6, 6> sub_mat_H = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
    size_t num_sel_features = 0;
    TicToc t_sel_feature;
    while (true)
    {
        if ((num_sel_features >= num_use_features) ||
            (all_feature_idx.size() == 0) ||
            (t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME))
            break;
        size_t num_rnd_que;
        std::priority_queue<FeatureWithScore, std::vector<FeatureWithScore>, std::less<FeatureWithScore> > heap_subset;
        while (heap_subset.size() < size_rnd_subset)
        {
            num_rnd_que = 0;
            size_t j;
            while (num_rnd_que < MAX_RANDOM_QUEUE_TIME)
            {
                j = (std::rand() % all_feature_idx.size());
                if (feature_visited[j] < int(num_sel_features))
                {
                    feature_visited[j] = int(num_sel_features);
                    break;
                }
                num_rnd_que++;
            }
            if (num_rnd_que >= MAX_RANDOM_QUEUE_TIME || t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME)
                break;

            size_t que_idx = all_feature_idx[j];
            const PointPlaneFeature &feature = all_features[que_idx];
            Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity();
            if (feature.type_ == 's')
                extractCov(laser_cloud_surf_cov[feature.laser_idx_].points[feature.idx_], cov_matrix);
            else if (feature.type_ == 's')
                extractCov(laser_cloud_corner_cov[feature.laser_idx_].points[feature.idx_], cov_matrix);
            Eigen::MatrixXd jaco;
            evaluateFeatJacobian(para_pose, feature, cov_matrix, jaco);

            double cur_det = common::logDet(sub_mat_H + jaco.transpose() * jaco, true);
            heap_subset.push(FeatureWithScore(que_idx, cur_det, jaco));
            if (heap_subset.size() >= size_rnd_subset)
            {
                const FeatureWithScore &fws = heap_subset.top();
                std::vector<size_t>::iterator iter = std::find(all_feature_idx.begin(), all_feature_idx.end(), fws.idx_);
                if (iter == all_feature_idx.end())
                {
                    std::cerr << "[goodFeatureSelct]: not exist feature idx !" << std::endl;
                    break;
                }

                const Eigen::MatrixXd &jaco = fws.jaco_;
                sub_mat_H += jaco.transpose() * jaco;

                size_t position = iter - all_feature_idx.begin();
                all_feature_idx.erase(all_feature_idx.begin() + position);
                feature_visited.erase(feature_visited.begin() + position);
                sel_feature_idx.push_back(fws.idx_);
                num_sel_features++;
                // printf("position: %lu, num: %lu\n", position, num_rnd_que);
                break;
            }
        }
        if (num_rnd_que >= MAX_RANDOM_QUEUE_TIME)
        {
            std::cerr << "[goodFeatureSelct]: early termination!" << std::endl;
            break;
        }
    }
    printf("logdet of selected sub H: %f\n", common::logDet(sub_mat_H));

    // {
    //     Eigen::Matrix<double, 6, 6> total_mat_H = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
    //     for (size_t que_idx = 0; que_idx < num_all_features; que_idx++)
    //     {
    //         const PointPlaneFeature &feature = all_features[que_idx];
    //         Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity();
    //         extractCov(laser_cloud_cov[feature.laser_idx_].points[feature.idx_], cov_matrix);
    //         Eigen::MatrixXd jaco;
    //         evaluateFeatJacobian(para_pose, feature, cov_matrix, jaco);
    //         total_mat_H += jaco.transpose() * jaco;
    //     }
    //     printf("logdet of the selected/complete H: %f(%f)\n", 
    //         common::logDet(sub_mat_H, true) ,common::logDet(total_mat_H, true));
    // }
    printf("good feature selection time: %fms\n", t_sel_feature.toc());
}

void writeFeature(const std::vector<size_t> &sel_feature_idx,
                  const std::vector<PointPlaneFeature> &surf_map_features)
{
    pcl::PCDWriter pcd_writer;

    common::PointICloud origin_map_feature;
    for (const PointPlaneFeature &feature : surf_map_features)
    {
        common::PointI point;
        point.x = feature.point_[0];
        point.z = feature.point_[1];
        point.y = feature.point_[2];
        point.intensity = feature.laser_idx_;
        origin_map_feature.points.push_back(point);
    }
    origin_map_feature.height = 1;
    origin_map_feature.width = origin_map_feature.points.size();
    pcd_writer.write("/tmp/original_map_feature.pcd", origin_map_feature);

    common::PointICloud sel_map_feature;
    for (const size_t &idx : sel_feature_idx)
    {
        const PointPlaneFeature &feature = surf_map_features[idx];
        common::PointI point;
        point.x = feature.point_[0];
        point.z = feature.point_[1];
        point.y = feature.point_[2];
        point.intensity = feature.laser_idx_;
        sel_map_feature.points.push_back(point);
    }
    sel_map_feature.height = 1;
    sel_map_feature.width = sel_map_feature.points.size();
    pcd_writer.write("/tmp/sel_map_feature.pcd", sel_map_feature);
}

//
