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
#include "../factor/impl_loss_function.hpp"
#include "associate_uct.hpp"

// #define MAX_BUF_LENGTH 3
#define SURROUNDING_KF_RADIUS 50.0
#define GLOBALMAP_KF_RADIUS 1000.0
#define DISTANCE_KEYFRAMES 1.0
#define ORIENTATION_KEYFRAMES 3
#define MAX_FEATURE_SELECT_TIME 20 // 10ms
#define MAX_RANDOM_QUEUE_TIME 20
#define LOGDET_H_THRESHOLD 64 // TODO
#define GF_RATIO_SIGMA 1.20
#define LAMBDA_SCALAR 0.003 // TODO

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");
DEFINE_bool(with_ua, true, "with or without the awareness of uncertainty");
DEFINE_string(gf_method, "gd_float", "good feature selection method: gd_fix, gd_float, rnd, ds");
DEFINE_double(gf_ratio_ini, 0.2, "with or without the good features selection");

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

// ****************** other operation
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
    LidarMapPlaneNormFactor f(feature.point_, feature.coeffs_, cov_matrix);
    const double **param = new const double *[1];
    param[0] = para_pose;

    double *res = new double[1];
    double **jaco = new double *[1];
    jaco[0] = new double[3 * 7];
    f.Evaluate(param, res, jaco);
    Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > mat_jacobian(jaco[0]);
    mat_jaco = mat_jacobian.topLeftCorner<3, 6>();

    delete[] res;
    delete[] jaco;
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

    LidarMapPlaneNormFactor f(feature.point_, feature.coeffs_, cov_matrix);
    double **param = new double *[1];
    param[0] = pose_array;

    double *res = new double[1];
    double **jaco = new double *[1];
    jaco[0] = new double[3 * 7];
    f.Evaluate(param, res, jaco);
    Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> mat_jacobian(jaco[0]);
    mat_jaco = mat_jacobian.topLeftCorner<3, 6>();

    delete[] param;
    delete[] res;
    delete[] jaco;
}

void evalFullHessian(const pcl::KdTreeFLANN<PointIWithCov>::Ptr &kdtree_from_map,
                     const PointICovCloud &laser_map,
                     const PointICovCloud &laser_cloud,
                     const Pose &pose_local,
                     const char feature_type,
                     Eigen::Matrix<double, 6, 6> &mat_H,
                     int &feat_num)
{
    size_t num_all_features = laser_cloud.size();
    std::vector<PointPlaneFeature> all_features(num_all_features);
    // std::vector<Eigen::MatrixXd> v_jaco;
    for (size_t i = 0; i < num_all_features; i++) 
    {
        size_t n_neigh = 5;
        bool b_match = true;
        if (feature_type == 's')
        {
            b_match = f_extract.matchSurfPointFromMap(kdtree_from_map,
                                                      laser_map,
                                                      laser_cloud.points[i],
                                                      pose_local,
                                                      all_features[i],
                                                      i,
                                                      n_neigh,
                                                      false);
        } else if (feature_type == 'c')
        {
            b_match = f_extract.matchCornerPointFromMap(kdtree_from_map,
                                                        laser_map,
                                                        laser_cloud.points[i],
                                                        pose_local,
                                                        all_features[i],
                                                        i,
                                                        n_neigh,
                                                        false);
        }
        if (!b_match) continue;
        Eigen::Matrix3d cov_matrix;
        extractCov(laser_cloud.points[i], cov_matrix);
        Eigen::MatrixXd jaco;
        evaluateFeatJacobianMatching(pose_local, all_features[i], cov_matrix, jaco);
        mat_H = mat_H + jaco.transpose() * jaco;
        // v_jaco.push_back(jaco);
        feat_num++;
    }
    // std::ofstream fout((OUTPUT_FOLDER + "jacobian.txt").c_str(), std::ios::out);
    // fout << "jacobian" << std::endl;
    // fout.precision(8);
    // for (size_t i = 0; i < v_jaco.size(); i++)
    //     fout << v_jaco[i] << std::endl;
    // fout.close();
}

double goodFeatureMatching(const pcl::KdTreeFLANN<PointIWithCov>::Ptr &kdtree_from_map,
                           const PointICovCloud &laser_map,
                           const PointICovCloud &laser_cloud,
                           const Pose &pose_local,
                           std::vector<PointPlaneFeature> &all_features,
                           std::vector<size_t> &sel_feature_idx,
                           const char feature_type,
                           const string gf_method,
                           const double gf_ratio = 0.2,
                           const double lambda = 10.0,
                           bool ratio_change_flag = false)
{
    size_t num_all_features = laser_cloud.size();
    all_features.resize(num_all_features);

    std::vector<size_t> all_feature_idx(num_all_features);
    std::vector<int> feature_visited(num_all_features, -1);
    std::iota(all_feature_idx.begin(), all_feature_idx.end(), 0);

    size_t num_use_features;
    double cur_gf_ratio = gf_ratio;
    double pre_gf_ratio = cur_gf_ratio;
    num_use_features = static_cast<size_t>(num_all_features * cur_gf_ratio);
    sel_feature_idx.resize(num_use_features);
    size_t num_sel_features = 0;

    size_t n_neigh = 5;
    bool b_match;

    if (gf_method == "fps")
    {
        PointICovCloud laser_cloud_ds;
        size_t k = 0;
        feature_visited[k] = 1;
        PointIWithCov point_old = laser_cloud.points[k]; 
        laser_cloud_ds.push_back(point_old);
        std::vector<float> dist(num_all_features, 1e5);
        for (size_t i = 1; i < num_use_features; i++)
        {
            float best = -1;
            size_t best_j = 1;
            for (size_t j = 0; j < num_all_features; j++)
            {
                if (feature_visited[j] == 1) continue;
                const PointIWithCov &point_new = laser_cloud.points[j];
                float d = sqrt(common::sqrSum(point_old.x - point_new.x,
                                              point_old.y - point_new.y,
                                              point_old.z - point_new.z));
                float d2 = std::min(d, dist[j]);
                dist[j] = d2;
                best = d2 > best ? d2 : best;
                best_j = d2 > best ? j : best_j;
            }
            point_old = laser_cloud.points[best_j];
            feature_visited[best_j] = 1;
            laser_cloud_ds.push_back(point_old);
        }

        // pcl::VoxelGridCovarianceMLOAM<PointIWithCov> down_size_filter_cloud;
        // down_size_filter_cloud.setLeafSize(1.0, 1.0, 1.0);
        // down_size_filter_cloud.setInputCloud(boost::make_shared<PointICovCloud>(laser_cloud));
        // down_size_filter_cloud.filter(laser_cloud_ds);
        // num_use_features = laser_cloud_ds.size();
        // sel_feature_idx.resize(num_use_features);

        for (size_t que_idx = 0; que_idx < num_use_features; que_idx++)
        {
            b_match = true;
            if (feature_type == 's')
            {
                b_match = f_extract.matchSurfPointFromMap(kdtree_from_map,
                                                          laser_map,
                                                          laser_cloud_ds.points[que_idx],
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
                                                            laser_cloud_ds.points[que_idx],
                                                            pose_local,
                                                            all_features[que_idx],
                                                            que_idx,
                                                            n_neigh,
                                                            false);
            }
            if (b_match)
            {
                sel_feature_idx[num_sel_features] = que_idx;
                num_sel_features++;
            }            
        }
    } else
    if (gf_method == "rnd")
    {
        size_t num_rnd_que;
        TicToc t_sel_feature;
        while (true)
        {
            if (num_sel_features >= num_use_features || 
                t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME ||
                all_feature_idx.size() == 0 || 
                num_rnd_que >= MAX_RANDOM_QUEUE_TIME)
                    break;
            size_t j;
            num_rnd_que = 0;
            while (num_rnd_que < MAX_RANDOM_QUEUE_TIME)
            {
                j = (std::rand() % all_feature_idx.size());
                if (feature_visited[j] == -1)
                {
                    feature_visited[j] = 0;
                    break;
                }
                num_rnd_que++;
            }
            b_match = true;
            size_t que_idx = all_feature_idx[j];
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
                sel_feature_idx[num_sel_features] = que_idx;
                num_sel_features++;
            } else
            {
                all_feature_idx.erase(all_feature_idx.begin() + j);
                feature_visited.erase(feature_visited.begin() + j);
            }
        }
    } else
    {
        if (gf_method == "gd_fix") ratio_change_flag = true;

        Eigen::Matrix<double, 6, 6> sub_mat_H = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
        double pre_cost = 0;
        double cur_cost = 0;
        stringstream ss;
        while (true)
        {
            TicToc t_sel_feature;
            size_t num_rnd_que;
            while (true)
            {
                if ((num_sel_features >= num_use_features) ||
                    (all_feature_idx.size() == 0) ||
                    (t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME))
                        break;

                size_t size_rnd_subset = static_cast<size_t>(1.0 * num_all_features / num_use_features);
                // LOG_EVERY_N(INFO, 20) << "[goodFeatureMatching] size of matrix subset: " << size_rnd_subset;

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
                        b_match = true;
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
                        } else if (feature_type == 'c')
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
                    Eigen::Matrix3d cov_matrix;
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
                        sel_feature_idx[num_sel_features] = fws.idx_;
                        num_sel_features++;
                        // printf("position: %lu, num: %lu\n", position, num_rnd_que);
                        break;
                    }
                }
                if (num_rnd_que >= MAX_RANDOM_QUEUE_TIME || t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME)
                    break;
            }
            if (!ratio_change_flag) break;

            if (num_rnd_que >= MAX_RANDOM_QUEUE_TIME || t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME)
            {
                std::cerr << "[goodFeatureMatching]: early termination!" << std::endl;
                LOG(INFO) << "early termination: " << num_rnd_que << ", " << t_sel_feature.toc();
                num_sel_features = static_cast<size_t>(num_all_features * pre_gf_ratio);
                break;
            }

            cur_cost = common::logDet(sub_mat_H) - lambda * 1e-3 * num_use_features;
            ss << cur_cost << "(" << cur_gf_ratio << ") ";
            if (cur_cost <= pre_cost) 
            {
                std::cout << ss.str() << std::endl;
                num_sel_features = static_cast<size_t>(num_all_features * pre_gf_ratio);
                break;
            } else
            {
                pre_cost = cur_cost;
                pre_gf_ratio = std::min(1.0, cur_gf_ratio);
                cur_gf_ratio = GF_RATIO_SIGMA * cur_gf_ratio;
                if (cur_gf_ratio > 1.0) break;
                num_use_features = static_cast<size_t>(num_all_features * cur_gf_ratio);
                sel_feature_idx.resize(num_use_features);
            }
        }
    } 

    sel_feature_idx.resize(num_sel_features);
    printf("num of all features: %lu, sel features: %lu\n", num_all_features, num_sel_features);
    // printf("logdet of selected sub H: %f\n", common::logDet(sub_mat_H));
    return pre_gf_ratio;
}

void writeFeature(const PointICovCloud &laser_cloud,
                  const std::vector<size_t> &sel_feature_idx,
                  const std::vector<PointPlaneFeature> &surf_map_features)
{
    std::string filename1 = OUTPUT_FOLDER + "/pcd/map_feature_" + "origin" + "_" + std::to_string((float)FLAGS_gf_ratio_ini) + ".pcd";
    std::string filename2 = OUTPUT_FOLDER + "/pcd/map_feature_" + FLAGS_gf_method + "_" + std::to_string((float)FLAGS_gf_ratio_ini) + ".pcd";

    pcl::PCDWriter pcd_writer;
    pcd_writer.write(filename1.c_str(), laser_cloud);

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
    pcd_writer.write(filename2.c_str(), sel_map_feature);
}

//
