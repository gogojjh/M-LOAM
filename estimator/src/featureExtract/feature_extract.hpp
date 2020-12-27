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

#ifndef FEATURE_EXTRACT_HPP
#define FEATURE_EXTRACT_HPP

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <cmath>
#include <map>
#include <omp.h>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "common/types/type.h"
#include "common/algos/math.hpp"

#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

#include "mloam_pcl/point_with_time.hpp"

#define EDGE_THRESHOLD 0.1

using namespace common;

class compObject
{
public:
    float *cloud_curvature;
    bool operator()(int i, int j) { return (cloud_curvature[i] < cloud_curvature[j]); }
};

class FeatureExtract
{
public:
    FeatureExtract() {}

    void findStartEndAngle(const PointCloud &laser_cloud_in, 
                           float &start_ori, 
                           float &end_ori);

    void findStartEndTime(const PointITimeCloud &laser_cloud_in,
                          float &start_time, 
                          float &end_time);

    void calTimestamp(const PointCloud &laser_cloud_in,
                      PointICloud &laser_cloud_out);

    void calTimestamp(const PointITimeCloud &laser_cloud_in,
                      PointICloud &laser_cloud_out);

    void extractCloud(const PointICloud &laser_cloud_in,
                      const ScanInfo &scan_info,
                      cloudFeature &cloud_feature);

    template <typename PointType>
    void matchCornerFromScan(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_corner_from_scan,
                             const typename pcl::PointCloud<PointType> &cloud_scan, 
                             const typename pcl::PointCloud<PointType> &cloud_data,
                             const Pose &pose_local,
                             std::vector<PointPlaneFeature> &features);
    
    template <typename PointType>
    void matchSurfFromScan(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_surf_from_scan,
                           const typename pcl::PointCloud<PointType> &cloud_scan, 
                           const typename pcl::PointCloud<PointType> &cloud_data,
                           const Pose &pose_local, 
                           std::vector<PointPlaneFeature> &features);
    
    template <typename PointType>
    void matchCornerFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_corner_from_map,
                            const typename pcl::PointCloud<PointType> &cloud_map, 
                            const typename pcl::PointCloud<PointType> &cloud_data,
                            const Pose &pose_local, 
                            std::vector<PointPlaneFeature> &features,
                            const size_t &N_NEIGH = 5, 
                            const bool &CHECK_FOV = true);

    template <typename PointType>
    void matchSurfFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_surf_from_map,
                          const typename pcl::PointCloud<PointType> &cloud_map,
                          const typename pcl::PointCloud<PointType> &cloud_data,
                          const Pose &pose_local,
                          std::vector<PointPlaneFeature> &features,
                          const size_t &N_NEIGH = 5,
                          const bool &CHECK_FOV = true);

    template <typename PointType>
    bool matchCornerPointFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_corner_from_map,
                                 const typename pcl::PointCloud<PointType> &cloud_map,
                                 const PointType &point_ori,
                                 const Pose &pose_local,
                                 PointPlaneFeature &feature,
                                 const size_t &idx,
                                 const size_t &N_NEIGH = 5,
                                 const bool &CHECK_FOV = true);

    template <typename PointType>
    bool matchSurfPointFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_surf_from_map,
                               const typename pcl::PointCloud<PointType> &cloud_map,
                               const PointType &point_ori,
                               const Pose &pose_local,
                               PointPlaneFeature &feature,
                               const size_t &idx,
                               const size_t &N_NEIGH = 5,
                               const bool &CHECK_FOV = true);
};

template <typename PointType>
void FeatureExtract::matchCornerFromScan(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_corner_from_scan,
                                         const typename pcl::PointCloud<PointType> &cloud_scan,
                                         const typename pcl::PointCloud<PointType> &cloud_data,
                                         const Pose &pose_local,
                                         std::vector<PointPlaneFeature> &features)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        std::cerr << "[FeatureExtract] Point does not have intensity field!" << std::endl;
        exit(EXIT_FAILURE);
    }

    size_t cloud_size = cloud_data.points.size();
    features.resize(cloud_size);
    size_t cloud_cnt = 0;
    
    PointType point_sel;
    std::vector<int> point_search_ind;
    std::vector<float> point_search_sqdis;
    for (size_t i = 0; i < cloud_data.points.size(); i++)
    {
        // not consider distortion
        TransformToStart(cloud_data.points[i], point_sel, pose_local, false, SCAN_PERIOD);
        kdtree_corner_from_scan->nearestKSearch(point_sel, 1, point_search_ind, point_search_sqdis);

        int closest_point_ind = -1, min_point_ind2 = -1;
        if (point_search_sqdis[0] < DISTANCE_SQ_THRESHOLD)
        {
            closest_point_ind = point_search_ind[0];
            int closest_point_scan_id = int(cloud_scan.points[closest_point_ind].intensity);

            float min_point_sqdis2 = DISTANCE_SQ_THRESHOLD;
            // search in the direction of increasing scan line
            for (int j = closest_point_ind + 1; j < (int)cloud_scan.points.size(); j++)
            {
                // if in the same scan line, continue
                if (int(cloud_scan.points[j].intensity) <= closest_point_scan_id)
                    continue;
                // if not in nearby scans, end the loop
                if (int(cloud_scan.points[j].intensity) > (closest_point_scan_id + NEARBY_SCAN))
                    break;
                float point_sqdis = sqrSum(cloud_scan.points[j].x - point_sel.x,
                                           cloud_scan.points[j].y - point_sel.y,
                                           cloud_scan.points[j].z - point_sel.z);
                if (point_sqdis < min_point_sqdis2)
                {
                    // find nearer point
                    min_point_sqdis2 = point_sqdis;
                    min_point_ind2 = j;
                }
            }

            // search in the direction of decreasing scan line
            for (int j = closest_point_ind - 1; j >= 0; j--)
            {
                // if in the same scan line, continue
                if (int(cloud_scan.points[j].intensity) >= closest_point_scan_id)
                    continue;
                // if not in nearby scans, end the loop
                if (int(cloud_scan.points[j].intensity) < (closest_point_scan_id - NEARBY_SCAN))
                    break;
                float point_sqdis = sqrSum(cloud_scan.points[j].x - point_sel.x,
                                           cloud_scan.points[j].y - point_sel.y,
                                           cloud_scan.points[j].z - point_sel.z);
                if (point_sqdis < min_point_sqdis2)
                {
                    // find nearer point
                    min_point_sqdis2 = point_sqdis;
                    min_point_ind2 = j;
                }
            }
        }

        if (min_point_ind2 >= 0) // both closest_point_ind and min_point_ind2 is valid
        {
            // Eigen::Vector3f X0(point_sel.x, point_sel.y, point_sel.z);
            // Eigen::Vector3f X1(cloud_scan.points[closest_point_ind].x,
            //                    cloud_scan.points[closest_point_ind].y,
            //                    cloud_scan.points[closest_point_ind].z);
            // Eigen::Vector3f X2(cloud_scan.points[min_point_ind2].x,
            //                    cloud_scan.points[min_point_ind2].y,
            //                    cloud_scan.points[min_point_ind2].z);
            // Eigen::Vector3f n = (X1 - X0).cross(X2 - X0);
            // Eigen::Vector3f w2 = n / n.norm();
            // Eigen::Vector3f w1 = w2.cross(X2 - X1);
            // w1.normalized();

            // float ld_1 = n.norm() / (X1 - X2).norm(); // distance
            // float ld_2 = 0.0;
            // float s = 1 - 0.9f * fabs(ld_1);

            // float ld_p1 = -w1.dot(X1);
            // float ld_p2 = -w2.dot(X1);

            // float ld_p1 = -(w1.x() * point_sel.x + w1.y() * point_sel.y + w1.z() * point_sel.z - ld_1);
            // float ld_p2 = -(w2.x() * point_sel.x + w2.y() * point_sel.y + w2.z() * point_sel.z - ld_2);

            // Eigen::Vector4d coeff1(w1.x(), w1.y(), w1.z(), ld_p1);
            // Eigen::Vector4d coeff2(w2.x(), w2.y(), w2.z(), ld_p2);

            // PointPlaneFeature feature1, feature2;
            // feature1.idx_ = i;
            // feature1.point_ = Eigen::Vector3d{cloud_data.points[i].x, cloud_data.points[i].y, cloud_data.points[i].z};
            // feature1.coeffs_ = coeff1 * 0.5;
            // feature1.type_ = 'c';
            // features[cloud_cnt] = feature1;
            // cloud_cnt++;

            // feature2.idx_ = i;
            // feature2.point_ = Eigen::Vector3d{cloud_data.points[i].x, cloud_data.points[i].y, cloud_data.points[i].z};
            // feature2.coeffs_ = coeff2 * 0.5;
            // feature2.type_ = 'c';
            // features[cloud_cnt] = feature2;
            // cloud_cnt++;

            // Eigen::Vector4d coeff(w1.x(), w1.y(), w1.z(), ld_p1);
            // PointPlaneFeature feature;
            // feature.idx_ = i;
            // feature.point_ = Eigen::Vector3d{cloud_data.points[i].x, cloud_data.points[i].y, cloud_data.points[i].z};
            // feature.coeffs_ = coeff;
            // features[cloud_cnt] = feature;
            // cloud_cnt++;

            Eigen::Matrix<double, 6, 1> coeff;
            coeff(0) = cloud_scan.points[closest_point_ind].x,
            coeff(1) = cloud_scan.points[closest_point_ind].y,
            coeff(2) = cloud_scan.points[closest_point_ind].z;
            coeff(3) = cloud_scan.points[min_point_ind2].x,
            coeff(4) = cloud_scan.points[min_point_ind2].y,
            coeff(5) = cloud_scan.points[min_point_ind2].z;
            PointPlaneFeature feature;
            feature.idx_ = i;
            feature.point_ = Eigen::Vector3d{cloud_data.points[i].x, cloud_data.points[i].y, cloud_data.points[i].z};
            feature.coeffs_ = coeff;
            features[cloud_cnt] = feature;
            cloud_cnt++;
        }
    }
    features.resize(cloud_cnt);
}

template <typename PointType>
void FeatureExtract::matchSurfFromScan(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_surf_from_scan,
                                       const typename pcl::PointCloud<PointType> &cloud_scan,
                                       const typename pcl::PointCloud<PointType> &cloud_data,
                                       const Pose &pose_local,
                                       std::vector<PointPlaneFeature> &features)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        std::cerr << "[FeatureExtract] Point does not have intensity field!" << std::endl;
        exit(EXIT_FAILURE);
    }
    features.clear();
    PointType point_sel;
    std::vector<int> point_search_ind;
    std::vector<float> point_search_sqdis;
    for (size_t i = 0; i < cloud_data.points.size(); i++)
    {
        // not consider distortion
        TransformToStart(cloud_data.points[i], point_sel, pose_local, false, SCAN_PERIOD);
        kdtree_surf_from_scan->nearestKSearch(point_sel, 1, point_search_ind, point_search_sqdis);

        int closest_point_ind = -1, min_point_ind2 = -1, min_point_ind3 = -1;
        if (point_search_sqdis[0] < DISTANCE_SQ_THRESHOLD)
        {
            closest_point_ind = point_search_ind[0];
            // get closest point's scan ID
            int closest_point_scan_id = int(cloud_scan.points[closest_point_ind].intensity);
            float min_point_sqdis2 = DISTANCE_SQ_THRESHOLD, min_point_sqdis3 = DISTANCE_SQ_THRESHOLD;

            // search in the direction of increasing scan line
            for (int j = closest_point_ind + 1; j < (int)cloud_scan.points.size(); j++)
            {
                // if not in nearby scans, end the loop
                if (int(cloud_scan.points[j].intensity) > (closest_point_scan_id + NEARBY_SCAN))
                    break;
                float point_sqdis = sqrSum(cloud_scan.points[j].x - point_sel.x,
                                           cloud_scan.points[j].y - point_sel.y,
                                           cloud_scan.points[j].z - point_sel.z);
                // if in the same or lower scan line
                if (int(cloud_scan.points[j].intensity) <= closest_point_scan_id && point_sqdis < min_point_sqdis2)
                {
                    min_point_sqdis2 = point_sqdis;
                    min_point_ind2 = j;
                }
                // if in the higher scan line
                else if (int(cloud_scan.points[j].intensity) > closest_point_scan_id && point_sqdis < min_point_sqdis3)
                {
                    min_point_sqdis3 = point_sqdis;
                    min_point_ind3 = j;
                }
            }

            // search in the direction of decreasing scan line
            for (int j = closest_point_ind - 1; j >= 0; j--)
            {
                // if not in nearby scans, end the loop
                if (int(cloud_scan.points[j].intensity) < (closest_point_scan_id - NEARBY_SCAN))
                    break;
                float point_sqdis = sqrSum(cloud_scan.points[j].x - point_sel.x,
                                           cloud_scan.points[j].y - point_sel.y,
                                           cloud_scan.points[j].z - point_sel.z);
                // if in the same or higher scan line
                if (int(cloud_scan.points[j].intensity) >= closest_point_scan_id && point_sqdis < min_point_sqdis2)
                {
                    min_point_sqdis2 = point_sqdis;
                    min_point_ind2 = j;
                }
                else if (int(cloud_scan.points[j].intensity) < closest_point_scan_id && point_sqdis < min_point_sqdis3)
                {
                    // find nearer point
                    min_point_sqdis3 = point_sqdis;
                    min_point_ind3 = j;
                }
            }

            if (min_point_ind2 >= 0 && min_point_ind3 >= 0)
            {
                Eigen::Vector3f last_point_j(cloud_scan.points[closest_point_ind].x,
                                             cloud_scan.points[closest_point_ind].y,
                                             cloud_scan.points[closest_point_ind].z);
                Eigen::Vector3f last_point_l(cloud_scan.points[min_point_ind2].x,
                                             cloud_scan.points[min_point_ind2].y,
                                             cloud_scan.points[min_point_ind2].z);
                Eigen::Vector3f last_point_m(cloud_scan.points[min_point_ind3].x,
                                             cloud_scan.points[min_point_ind3].y,
                                             cloud_scan.points[min_point_ind3].z);
                Eigen::Vector3f w = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
                w.normalize();
                float negative_OA_dot_norm = -w.dot(last_point_j);
                float pd2 = -(w.x() * point_sel.x + w.y() * point_sel.y + w.z() * point_sel.z + negative_OA_dot_norm); // distance
                float s = 1 - 0.9f * fabs(pd2) / sqrt(sqrSum(point_sel.x, point_sel.y, point_sel.z));

                Eigen::Vector4d coeff(w.x(), w.y(), w.z(), negative_OA_dot_norm);
                PointPlaneFeature feature;
                feature.idx_ = i;
                feature.point_ = Eigen::Vector3d{cloud_data.points[i].x, cloud_data.points[i].y, cloud_data.points[i].z};
                feature.coeffs_ = coeff;
                feature.type_ = 's';
                features.push_back(feature);
            }
        }
    }
}

template <typename PointType>
void FeatureExtract::matchCornerFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_corner_from_map,
                                        const typename pcl::PointCloud<PointType> &cloud_map,
                                        const typename pcl::PointCloud<PointType> &cloud_data,
                                        const Pose &pose_local,
                                        std::vector<PointPlaneFeature> &features,
                                        const size_t &N_NEIGH,
                                        const bool &CHECK_FOV)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        std::cerr << "[FeatureExtract] Point does not have intensity field!" << std::endl;
        exit(EXIT_FAILURE);
    }
    features.clear();
    std::vector<int> point_search_idx(N_NEIGH, 0);
    std::vector<float> point_search_sq_dis(N_NEIGH, 0);

    // extract edge coefficients and correspondences from edge map
    size_t cloud_size = cloud_data.points.size();
    features.resize(cloud_size * 2);
    size_t cloud_cnt = 0;
    PointType point_ori, point_sel;
    int num_neighbors = N_NEIGH;
    for (size_t i = 0; i < cloud_size; i++)
    {
        point_ori = cloud_data.points[i];
        pointAssociateToMap(point_ori, point_sel, pose_local);
        kdtree_corner_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
        if (point_search_sq_dis[num_neighbors - 1] < MIN_MATCH_SQ_DIS)
        {
            // calculate the coefficients of edge points
            std::vector<Eigen::Vector3f> near_corners;
            Eigen::Vector3f center(0, 0, 0); // mean value
            for (int j = 0; j < num_neighbors; j++)
            {
                Eigen::Vector3f tmp(cloud_map.points[point_search_idx[j]].x,
                                    cloud_map.points[point_search_idx[j]].y,
                                    cloud_map.points[point_search_idx[j]].z);
                center += tmp;
                near_corners.push_back(tmp);
            }
            center /= (1.0 * num_neighbors);
            Eigen::Matrix3f cov_mat = Eigen::Matrix3f::Zero();
            for (int j = 0; j < num_neighbors; j++)
            {
                Eigen::Vector3f tmp_zero_mean = near_corners[j] - center;
                cov_mat += tmp_zero_mean * tmp_zero_mean.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(cov_mat);

            // if is indeed line feature
            // note Eigen library sort eigenvalues in increasing order
            Eigen::Vector3f unit_direction = esolver.eigenvectors().col(2);
            if (esolver.eigenvalues()[2] > 3 * esolver.eigenvalues()[1])
            {
                bool is_in_laser_fov = false;
                if (CHECK_FOV)
                {
                    PointType point_on_z_axis, point_on_z_axis_trans;
                    point_on_z_axis.x = 0.0;
                    point_on_z_axis.y = 0.0;
                    point_on_z_axis.z = 10.0;
                    pointAssociateToMap(point_on_z_axis, point_on_z_axis_trans, pose_local);
                    float squared_side1 = sqrSum(pose_local.t_(0) - point_sel.x,
                                                 pose_local.t_(1) - point_sel.y,
                                                 pose_local.t_(2) - point_sel.z);
                    float squared_side2 = sqrSum(point_on_z_axis_trans.x - point_sel.x,
                                                 point_on_z_axis_trans.y - point_sel.y,
                                                 point_on_z_axis_trans.z - point_sel.z);

                    float check1 = 100.0f + squared_side1 - squared_side2 - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                    float check2 = 100.0f + squared_side1 - squared_side2 + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                    // within +-60 degree
                    if (check1 < 0 && check2 > 0)
                        is_in_laser_fov = true;
                }
                else
                {
                    is_in_laser_fov = true;
                }
                if (is_in_laser_fov)
                {
                    // Eigen::Vector3f X0(point_sel.x, point_sel.y, point_sel.z);
                    // Eigen::Vector3f point_on_line = center;
                    // Eigen::Vector3f X1, X2;
                    // X1 = 0.1 * unit_direction + point_on_line;
                    // X2 = -0.1 * unit_direction + point_on_line;

                    // Eigen::Vector3f n = (X1 - X0).cross(X2 - X0);
                    // Eigen::Vector3f w2 = n / n.norm();
                    // Eigen::Vector3f w1 = w2.cross(X2 - X1);
                    // w1.normalized();

                    // float ld_1 = n.norm() / (X1 - X2).norm(); // the point-to-edge distance between point to plane
                    // float ld_2 = 0.0;
                    // float s = 1 - 0.9f * fabs(ld_1);

                    // float ld_p1 = -w1.dot(X1);
                    // float ld_p2 = -w2.dot(X1);

                    // float ld_p1 = -(w1.x() * point_sel.x + w1.y() * point_sel.y + w1.z() * point_sel.z - ld_1);
                    // float ld_p2 = -(w2.x() * point_sel.x + w2.y() * point_sel.y + w2.z() * point_sel.z - ld_2);

                    // Eigen::Vector4d coeff1(w1.x(), w1.y(), w1.z(), ld_p1);
                    // Eigen::Vector4d coeff2(w2.x(), w2.y(), w2.z(), ld_p2);

                    // PointPlaneFeature feature1, feature2;

                    // feature1.idx_ = i;
                    // feature1.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                    // feature1.coeffs_ = coeff1 * 0.5;
                    // feature1.laser_idx_ = (size_t)point_ori.intensity;
                    // feature1.type_ = 'c';
                    // features[cloud_cnt] = feature1;
                    // cloud_cnt++;

                    // feature2.idx_ = i;
                    // feature2.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                    // feature2.coeffs_ = coeff2 * 0.5;
                    // feature2.laser_idx_ = (size_t)point_ori.intensity;
                    // feature2.type_ = 'c';
                    // features[cloud_cnt] = feature2;
                    // cloud_cnt++;

                    // Eigen::Vector4d coeff(w1.x(), w1.y(), w1.z(), ld_p1);
                    // PointPlaneFeature feature;
                    // feature.idx_ = i;
                    // feature.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                    // feature.coeffs_ = coeff;
                    // feature.laser_idx_ = (size_t)point_ori.intensity;
                    // feature.type_ = 'c';
                    // features[cloud_cnt] = feature;
                    // cloud_cnt++;

                    Eigen::Vector3f point_on_line = center;
                    Eigen::Vector3f X1, X2;
                    X1 = 0.1 * unit_direction + point_on_line;
                    X2 = -0.1 * unit_direction + point_on_line;

                    Eigen::Matrix<double, 6, 1> coeff;
                    coeff(0) = X1.x(),
                    coeff(1) = X1.y(),
                    coeff(2) = X1.z();
                    coeff(3) = X2.x(),
                    coeff(4) = X2.y(),
                    coeff(5) = X2.z();
                    PointPlaneFeature feature;
                    feature.idx_ = i;
                    feature.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                    feature.coeffs_ = coeff;
                    feature.laser_idx_ = (size_t)point_ori.intensity;
                    feature.type_ = 'c';
                    features[cloud_cnt] = feature;
                    cloud_cnt++;
                }
            }
        }
    }
    features.resize(cloud_cnt);
}

// should be performed once after several gradient descents
template <typename PointType>
void FeatureExtract::matchSurfFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_surf_from_map,
                                      const typename pcl::PointCloud<PointType> &cloud_map,
                                      const typename pcl::PointCloud<PointType> &cloud_data,
                                      const Pose &pose_local,
                                      std::vector<PointPlaneFeature> &features,
                                      const size_t &N_NEIGH,
                                      const bool &CHECK_FOV)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        std::cerr << "[FeatureExtract] Point does not have intensity field!" << std::endl;
        exit(EXIT_FAILURE);
    }
    features.clear();
    std::vector<int> point_search_idx(N_NEIGH, 0);
    std::vector<float> point_search_sq_dis(N_NEIGH, 0);
    Eigen::MatrixXf mat_A = Eigen::MatrixXf::Zero(N_NEIGH, 3);
    Eigen::MatrixXf mat_B = Eigen::MatrixXf::Constant(N_NEIGH, 1, -1);
    const int num_neighbors = N_NEIGH;

    size_t cloud_size = cloud_data.points.size();
    features.resize(cloud_size);
    size_t cloud_cnt = 0;
    PointType point_ori, point_sel;
    for (size_t i = 0; i < cloud_size; i++)
    {
        point_ori = cloud_data.points[i];
        pointAssociateToMap(point_ori, point_sel, pose_local);
        kdtree_surf_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
        if (point_search_sq_dis[num_neighbors - 1] < MIN_MATCH_SQ_DIS)
        {
            for (int j = 0; j < num_neighbors; j++)
            {
                mat_A(j, 0) = cloud_map.points[point_search_idx[j]].x;
                mat_A(j, 1) = cloud_map.points[point_search_idx[j]].y;
                mat_A(j, 2) = cloud_map.points[point_search_idx[j]].z;
            }
            Eigen::Vector3f norm = mat_A.colPivHouseholderQr().solve(mat_B);
            float negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            // check if a plane that coeff * [x, y, z, 1] <= MIN_MATCH_SQ_DIS
            bool plane_valid = true;
            for (int j = 0; j < num_neighbors; j++)
            {
                if (fabs(norm(0) * cloud_map.points[point_search_idx[j]].x +
                         norm(1) * cloud_map.points[point_search_idx[j]].y +
                         norm(2) * cloud_map.points[point_search_idx[j]].z + negative_OA_dot_norm) > MIN_PLANE_DIS)
                {
                    plane_valid = false;
                    break;
                }
            }

            if (plane_valid)
            {
                bool is_in_laser_fov = false;
                if (CHECK_FOV)
                {
                    PointType transform_pos;
                    PointType point_on_z_axis, point_on_z_axis_trans;
                    point_on_z_axis.x = 0.0;
                    point_on_z_axis.y = 0.0;
                    point_on_z_axis.z = 10.0;
                    pointAssociateToMap(point_on_z_axis, point_on_z_axis_trans, pose_local);
                    float squared_side1 = sqrSum(pose_local.t_(0) - point_sel.x,
                                                  pose_local.t_(1) - point_sel.y,
                                                  pose_local.t_(2) - point_sel.z);
                    float squared_side2 = sqrSum(point_on_z_axis_trans.x - point_sel.x,
                                                  point_on_z_axis_trans.y - point_sel.y,
                                                  point_on_z_axis_trans.z - point_sel.z);
                    float check1 = 100.0f + squared_side1 - squared_side2 - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                    float check2 = 100.0f + squared_side1 - squared_side2 + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                    // within +-60 degree
                    if (check1 < 0 && check2 > 0)
                        is_in_laser_fov = true;
                }
                else
                {
                    is_in_laser_fov = true;
                }
                if (is_in_laser_fov)
                {
                    // pd2 (distance) smaller, s larger
                    float pd2 = norm(0) * point_sel.x + norm(1) * point_sel.y + norm(2) * point_sel.z + negative_OA_dot_norm;
                    float s = 1 - 0.9f * fabs(pd2) / sqrt(sqrSum(point_sel.x, point_sel.y, point_sel.z));

                    Eigen::Vector4d coeff(norm(0), norm(1), norm(2), negative_OA_dot_norm);
                    PointPlaneFeature feature;
                    feature.idx_ = i;
                    feature.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                    feature.coeffs_ = coeff;
                    feature.laser_idx_ = (size_t)point_ori.intensity;
                    feature.type_ = 's';
                    features[cloud_cnt] = feature;
                    cloud_cnt++;
                }
            }
        }
    }
    features.resize(cloud_cnt);
}

template <typename PointType>
bool FeatureExtract::matchCornerPointFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_corner_from_map,
                                             const typename pcl::PointCloud<PointType> &cloud_map,
                                             const PointType &point_ori,
                                             const Pose &pose_local,
                                             PointPlaneFeature &feature,
                                             const size_t &idx,
                                             const size_t &N_NEIGH,
                                             const bool &CHECK_FOV)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        LOG(INFO) << "[FeatureExtract] Point does not have intensity field!";
        return false;
    }
    std::vector<int> point_search_idx(N_NEIGH, 0);
    std::vector<float> point_search_sq_dis(N_NEIGH, 0);
    int num_neighbors = N_NEIGH;

    PointType point_sel;
    pointAssociateToMap(point_ori, point_sel, pose_local);
    kdtree_corner_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
    if (point_search_sq_dis[num_neighbors - 1] < MIN_MATCH_SQ_DIS)
    {
        // calculate the coefficients of edge points
        std::vector<Eigen::Vector3f> near_corners;
        Eigen::Vector3f center(0, 0, 0); // mean value
        for (int j = 0; j < num_neighbors; j++)
        {
            Eigen::Vector3f tmp(cloud_map.points[point_search_idx[j]].x,
                                cloud_map.points[point_search_idx[j]].y,
                                cloud_map.points[point_search_idx[j]].z);
            center += tmp;
            near_corners.push_back(tmp);
        }
        center /= (1.0 * num_neighbors);

        Eigen::Matrix3f cov_mat = Eigen::Matrix3f::Zero();
        for (int j = 0; j < num_neighbors; j++)
        {
            Eigen::Vector3f tmp_zero_mean = near_corners[j] - center;
            cov_mat += tmp_zero_mean * tmp_zero_mean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(cov_mat);

        // if is indeed line feature
        // note Eigen library sort eigenvalues in increasing order
        Eigen::Vector3f unit_direction = esolver.eigenvectors().col(2);
        if (esolver.eigenvalues()[2] > 3 * esolver.eigenvalues()[1])
        {
            bool is_in_laser_fov = false;
            if (CHECK_FOV)
            {
                PointType point_on_z_axis, point_on_z_axis_trans;
                point_on_z_axis.x = 0.0;
                point_on_z_axis.y = 0.0;
                point_on_z_axis.z = 10.0;
                pointAssociateToMap(point_on_z_axis, point_on_z_axis_trans, pose_local);
                float squared_side1 = sqrSum(pose_local.t_(0) - point_sel.x,
                                             pose_local.t_(1) - point_sel.y,
                                             pose_local.t_(2) - point_sel.z);
                float squared_side2 = sqrSum(point_on_z_axis_trans.x - point_sel.x,
                                             point_on_z_axis_trans.y - point_sel.y,
                                             point_on_z_axis_trans.z - point_sel.z);
 
                float check1 = 100.0f + squared_side1 - squared_side2 - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                float check2 = 100.0f + squared_side1 - squared_side2 + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                // within +-60 degree
                if (check1 < 0 && check2 > 0)
                    is_in_laser_fov = true;
            }
            else
            {
                is_in_laser_fov = true;
            }
            if (is_in_laser_fov) // TODO
            {
                // Eigen::Vector3f point_on_line = center;
                // Eigen::Vector3f X0(point_sel.x, point_sel.y, point_sel.z);
                // Eigen::Vector3f X1, X2;
                // X1 = 0.1 * unit_direction + point_on_line;
                // X2 = -0.1 * unit_direction + point_on_line;

                // Eigen::Vector3f n = (X1 - X0).cross(X2 - X0);
                // Eigen::Vector3f w2 = n / n.norm();
                // Eigen::Vector3f w1 = w2.cross(X2 - X1);
                // w1.normalized();

                // float ld_1 = n.norm() / (X1 - X2).norm(); // the distance between point to plane
                // float ld_2 = 0.0;
                // float s = 1 - 0.9f * fabs(ld_1);

                // float ld_p1 = -w1.dot(X1);
                // float ld_p2 = -w2.dot(X1);

                // // float ld_p1 = -(w1.x() * point_sel.x + w1.y() * point_sel.y + w1.z() * point_sel.z - ld_1);
                // // float ld_p2 = -(w2.x() * point_sel.x + w2.y() * point_sel.y + w2.z() * point_sel.z - ld_2);

                // // Eigen::Vector4d coeff1(w1.x(), w1.y(), w1.z(), ld_p1);
                // // Eigen::Vector4d coeff2(w2.x(), w2.y(), w2.z(), ld_p2);

                // // feature1.idx_ = idx;
                // // feature1.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                // // feature1.coeffs_ = coeff1 * 0.5;
                // // feature1.laser_idx_ = (size_t)point_ori.intensity;
                // // feature1.type_ = 'c';

                // // feature2.idx_ = idx;
                // // feature2.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                // // feature2.coeffs_ = coeff2 * 0.5;
                // // feature2.laser_idx_ = (size_t)point_ori.intensity;
                // // feature2.type_ = 'c';

                // Eigen::Vector4d coeff(w1.x(), w1.y(), w1.z(), ld_p1);
                // feature.idx_ = idx;
                // feature.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                // feature.coeffs_ = coeff;
                // feature.laser_idx_ = (size_t)point_ori.intensity;
                // feature.type_ = 'c';
                // return true;

                Eigen::Vector3f point_on_line = center;
                Eigen::Vector3f X1, X2;
                X1 = 0.1 * unit_direction + point_on_line;
                X2 = -0.1 * unit_direction + point_on_line;

                Eigen::Matrix<double, 6, 1> coeff;
                coeff(0) = X1.x(),
                coeff(1) = X1.y(),
                coeff(2) = X1.z();
                coeff(3) = X2.x(),
                coeff(4) = X2.y(),
                coeff(5) = X2.z();
                feature.idx_ = idx;
                feature.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                feature.coeffs_ = coeff;
                feature.laser_idx_ = (size_t)point_ori.intensity;
                feature.type_ = 'c';
                return true;
            }
        }
    }
    return false;
}

template <typename PointType>
bool FeatureExtract::matchSurfPointFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_surf_from_map,
                                           const typename pcl::PointCloud<PointType> &cloud_map,
                                           const PointType &point_ori,
                                           const Pose &pose_local,
                                           PointPlaneFeature &feature,
                                           const size_t &idx,
                                           const size_t &N_NEIGH,
                                           const bool &CHECK_FOV)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        LOG(INFO) << "[FeatureExtract] Point does not have intensity field!";
        return false;
    }
    std::vector<int> point_search_idx(N_NEIGH, 0);
    std::vector<float> point_search_sq_dis(N_NEIGH, 0);
    Eigen::MatrixXf mat_A = Eigen::MatrixXf::Zero(N_NEIGH, 3);
    Eigen::MatrixXf mat_B = Eigen::MatrixXf::Constant(N_NEIGH, 1, -1);
    const int num_neighbors = N_NEIGH;

    PointType point_sel;
    pointAssociateToMap(point_ori, point_sel, pose_local);
    kdtree_surf_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
    if (point_search_sq_dis[num_neighbors - 1] < MIN_MATCH_SQ_DIS)
    {
        std::vector<bool> point_select(num_neighbors, true);
        for (int j = 0; j < num_neighbors; j++)
        {
            mat_A(j, 0) = cloud_map.points[point_search_idx[j]].x;
            mat_A(j, 1) = cloud_map.points[point_search_idx[j]].y;
            mat_A(j, 2) = cloud_map.points[point_search_idx[j]].z;
        }
        Eigen::Vector3f norm = mat_A.colPivHouseholderQr().solve(mat_B);
        float negative_OA_dot_norm = 1 / norm.norm();
        norm.normalize();

        bool plane_valid = true;
        for (int j = 0; j < num_neighbors; j++)
        {
            if (fabs(norm(0) * cloud_map.points[point_search_idx[j]].x +
                     norm(1) * cloud_map.points[point_search_idx[j]].y +
                     norm(2) * cloud_map.points[point_search_idx[j]].z + negative_OA_dot_norm) > MIN_PLANE_DIS)
            {
                plane_valid = false;
                break;
            }
        }

        if (plane_valid)
        {
            bool is_in_laser_fov = false;
            if (CHECK_FOV)
            {
                PointType transform_pos;
                PointType point_on_z_axis, point_on_z_axis_trans;
                point_on_z_axis.x = 0.0;
                point_on_z_axis.y = 0.0;
                point_on_z_axis.z = 10.0;
                pointAssociateToMap(point_on_z_axis, point_on_z_axis_trans, pose_local);
                float squared_side1 = sqrSum(pose_local.t_(0) - point_sel.x,
                                                pose_local.t_(1) - point_sel.y,
                                                pose_local.t_(2) - point_sel.z);
                float squared_side2 = sqrSum(point_on_z_axis_trans.x - point_sel.x,
                                                point_on_z_axis_trans.y - point_sel.y,
                                                point_on_z_axis_trans.z - point_sel.z);
                float check1 = 100.0f + squared_side1 - squared_side2 - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                float check2 = 100.0f + squared_side1 - squared_side2 + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                // within +-60 degree
                if (check1 < 0 && check2 > 0)
                    is_in_laser_fov = true;
            }
            else
            {
                is_in_laser_fov = true;
            }
            if (is_in_laser_fov)
            {
                // pd2 (distance) smaller, s larger
                float pd2 = norm(0) * point_sel.x + norm(1) * point_sel.y + norm(2) * point_sel.z + negative_OA_dot_norm;
                float s = 1 - 0.9f * fabs(pd2) / sqrt(sqrSum(point_sel.x, point_sel.y, point_sel.z));

                Eigen::Vector4d coeff(norm(0), norm(1), norm(2), negative_OA_dot_norm);
                feature.idx_ = idx;
                feature.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                feature.coeffs_ = coeff;
                feature.laser_idx_ = (size_t)point_ori.intensity;
                feature.type_ = 's';
                return true;
            }
        }
    }
    return false;
}

#endif
//
