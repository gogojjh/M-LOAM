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
#include <signal.h>

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
#include "../featureExtract/feature_extract.hpp"
#include "../factor/lidar_map_plane_norm_factor.hpp"
#include "../factor/lidar_plane_norm_factor.hpp"
#include "../factor/pose_local_parameterization.h"
#include "associate_uct.hpp"

#define SURROUNDING_KF_RADIUS 50.0
#define GLOBALMAP_KF_RADIUS 2000.0
#define DISTANCE_KEYFRAMES 0.5
#define ORIENTATION_KEYFRAMES 5
#define MAX_FEATURE_SELECT_TIME 30 // 10ms
#define MAX_RANDOM_QUEUE_TIME 20

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");
DEFINE_bool(with_ua, true, "with or without the awareness of uncertainty");
DEFINE_double(gf_ratio, 0.2, "with or without the good features selection");

FeatureExtract f_extract;

// ****************** main process of lidar mapper
void transformAssociateToMap();

void extractSurroundingKeyFrames();

void downsampleCurrentScan();

void scan2MapOptimization();

void transformUpdate();

void saveKeyframeAndInsertGraph();

void pubGlobalMap();

void pubPointCloud();

void pubOdometry();

void saveGlobalMap();

void cloudUCTAssociateToMap(const PointICovCloud &cloud_local, PointICovCloud &cloud_global,
                            const Pose &pose_global, const vector<Pose> &pose_ext);

void evalHessian(const ceres::CRSMatrix &jaco, Eigen::Matrix<double, 6, 6> &mat_H);

void evalDegenracy(const Eigen::Matrix<double, 6, 6> &mat_H, PoseLocalParameterization *local_parameterization);

// ****************** good feature selection
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

// ****************** good feature selection
void evaluateFeatJacobianMatching(const Pose &pose_local,
                                  const PointPlaneFeature &feature,
                                  const Eigen::Matrix3d &cov_matrix,
                                  Eigen::MatrixXd &mat_jaco)
{
    double pose_array[SIZE_POSE];
    pose_array[0] = pose_local.t_(0);
    pose_array[1] = pose_local.t_(1);
    pose_array[2] = pose_local.t_(2);
    pose_array[3] = pose_local.q_.x();
    pose_array[4] = pose_local.q_.y();
    pose_array[5] = pose_local.q_.z();
    pose_array[6] = pose_local.q_.w();

    LidarMapPlaneNormFactor *f = new LidarMapPlaneNormFactor(feature.point_, feature.coeffs_, cov_matrix);
    const double **param = new const double *[1];
    param[0] = pose_array;
    double *res = new double[1];
    double **jaco = new double *[1];
    jaco[0] = new double[3 * 7];
    f->Evaluate(param, res, jaco);
    Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> mat_jacobian(jaco[0]);
    mat_jaco = mat_jacobian.topLeftCorner<3, 6>();
}

// TODO: add feature matching
void goodFeatureSelect(const double *para_pose,
                       const PointICovCloud &laser_cloud_surf_cov,
                       const PointICovCloud &laser_cloud_corner_cov,
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
    LOG_EVERY_N(INFO, 100) << "[goodFeatureSelct] size of matrix subset: " << size_rnd_subset;

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
                extractCov(laser_cloud_surf_cov.points[feature.idx_], cov_matrix);
            else if (feature.type_ == 's')
                extractCov(laser_cloud_corner_cov.points[feature.idx_], cov_matrix);
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

// TODO:
void goodFeatureMatching(const pcl::KdTreeFLANN<PointIWithCov>::Ptr &kdtree_from_map,
                         const PointICovCloud &laser_map,
                         const PointICovCloud &laser_cloud,
                         const Pose &pose_local,
                         std::vector<PointPlaneFeature> &all_features,
                         std::vector<size_t> &sel_feature_idx,
                         const char feature_type,
                         const float &gf_ratio = 0.2)
{
    size_t num_all_features = laser_cloud.size();
    all_features.resize(num_all_features);

    std::vector<size_t> all_feature_idx(num_all_features);
    std::vector<int> feature_visited(num_all_features, -1);
    std::iota(all_feature_idx.begin(), all_feature_idx.end(), 0);

    size_t num_use_features;
    if (gf_ratio <= 0.0)
    {
        num_use_features = 100;
    }
    else
    {
        num_use_features = static_cast<size_t>(num_all_features * gf_ratio);
    }
    printf("num of all features: %lu, sel features: %lu\n", num_all_features, num_use_features);

    size_t size_rnd_subset = static_cast<size_t>(1.0 * num_all_features / num_use_features);
    LOG_EVERY_N(INFO, 100) << "[goodFeatureMatching] size of matrix subset: " << size_rnd_subset;

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
        std::priority_queue<FeatureWithScore, std::vector<FeatureWithScore>, std::less<FeatureWithScore>> heap_subset;
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
            if (all_features[que_idx].type_ == 'n')
            {
                size_t n_neigh = 5;
                bool b_match = true;
                if (feature_type == 's')
                {
                    b_match = f_extract.matchSurfPointFromMap(kdtree_from_map,
                                                              laser_map,
                                                              laser_cloud.points[que_idx],
                                                              pose_local,
                                                              all_features[que_idx],
                                                              que_idx,
                                                              n_neigh,
                                                              false);
                }
                else if (feature_type == 'c')
                {
                    b_match = f_extract.matchCornerPointFromMap(kdtree_from_map,
                                                                laser_map,
                                                                laser_cloud.points[que_idx],
                                                                pose_local,
                                                                all_features[que_idx],
                                                                que_idx,
                                                                n_neigh,
                                                                false);
                }
                if (b_match) 
                {

                }
                else // not found constraints or outlier constraints
                {
                    all_feature_idx.erase(all_feature_idx.begin() + j);
                    feature_visited.erase(feature_visited.begin() + j);
                    break;
                }
            }

            const PointPlaneFeature &feature = all_features[que_idx];
            Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity();
            extractCov(laser_cloud.points[que_idx], cov_matrix);
            Eigen::MatrixXd jaco;
            evaluateFeatJacobianMatching(pose_local, feature, cov_matrix, jaco);

            double cur_det = common::logDet(sub_mat_H + jaco.transpose() * jaco, true);
            heap_subset.push(FeatureWithScore(que_idx, cur_det, jaco));
            if (heap_subset.size() >= size_rnd_subset)
            {
                const FeatureWithScore &fws = heap_subset.top();
                std::vector<size_t>::iterator iter = std::find(all_feature_idx.begin(), all_feature_idx.end(), fws.idx_);
                if (iter == all_feature_idx.end())
                {
                    std::cerr << "[goodFeatureMatching]: not exist feature idx !" << std::endl;
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
        if (num_rnd_que >= MAX_RANDOM_QUEUE_TIME || t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME)
        {
            std::cerr << "[goodFeatureMatching]: early termination!" << std::endl;
            break;
        }
    }
    printf("logdet of selected sub H: %f\n", common::logDet(sub_mat_H));
}


// TODO: add feature matching
void goodFeatureSelectTest(const double *para_pose,
                           const PointICovCloud &laser_cloud_surf_cov,
                           const PointICovCloud &laser_cloud_corner_cov,
                           const std::vector<PointPlaneFeature> &all_features,
                           const size_t &num_all_features,
                           std::vector<size_t> sel_feature_idx,
                           const float &gf_ratio = 0.2)
{
    Eigen::Matrix<double, 6, 6> total_mat_H = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
    double total_det;
    for (size_t que_idx = 0; que_idx < num_all_features; que_idx++)
    {
        const PointPlaneFeature &feature = all_features[que_idx];
        Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity();
        if (feature.type_ == 's')
            extractCov(laser_cloud_surf_cov.points[feature.idx_], cov_matrix);
        else if (feature.type_ == 'c')
            extractCov(laser_cloud_corner_cov.points[feature.idx_], cov_matrix);
        Eigen::MatrixXd jaco;
        evaluateFeatJacobian(para_pose, feature, cov_matrix, jaco);
        total_mat_H += jaco.transpose() * jaco;
    }
    total_det = common::logDet(total_mat_H, true);

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

    size_t size_rnd_subset = static_cast<size_t>(float(num_all_features) / float(num_use_features) * 1.0);
    Eigen::Matrix<double, 6, 6> sub_mat_H = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
    size_t num_sel_features = 0;
    TicToc t_sel_feature;
    double cur_det;
    while (true)
    {
        if (((num_sel_features >= num_use_features) && (cur_det >= total_det + std::log(0.8))) ||
            (all_feature_idx.size() == 0) ||
            (t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME))
            break;
        size_t num_rnd_que;
        std::priority_queue<FeatureWithScore, std::vector<FeatureWithScore>, std::less<FeatureWithScore>> heap_subset;
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
                extractCov(laser_cloud_surf_cov.points[feature.idx_], cov_matrix);
            else if (feature.type_ == 'c')
                extractCov(laser_cloud_corner_cov.points[feature.idx_], cov_matrix);
            Eigen::MatrixXd jaco;
            evaluateFeatJacobian(para_pose, feature, cov_matrix, jaco);

            cur_det = common::logDet(sub_mat_H + jaco.transpose() * jaco, true);
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
    // printf("logdet of selected sub H: %f\n", common::logDet(sub_mat_H));
    // printf("good feature selection time: %fms\n", t_sel_feature.toc());

    // complete H, threshold, |F|, |S|, |S|/|F|
    std::cout << total_det << " " << total_det + std::log(0.8) << " "
              << all_features.size() << " "
              << sel_feature_idx.size() << " "
              << (float)sel_feature_idx.size() * 1.0 / all_features.size() << std::endl;

    LOG_EVERY_N(INFO, 1) << total_det << " " << total_det + std::log(0.8) << " "
                         << all_features.size() << " "
                         << sel_feature_idx.size() << " "
                         << (float)sel_feature_idx.size() * 1.0 / all_features.size();
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
