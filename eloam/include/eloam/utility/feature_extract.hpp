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

#include <iostream>
#include <cmath>
#include <map>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

template <typename PointType>
inline void pointAssociateToMap(const PointType &pi, PointType &po, const Eigen::Matrix4f &T)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        std::cerr << "[pointAssociateToMap] Point does not have intensity field!" << std::endl;
        exit(EXIT_FAILURE);
    }
    po = pi;
    Eigen::Vector3f point_curr(pi.x, pi.y, pi.z);
    Eigen::Vector3f point_trans = T.block<3, 3>(0, 0) * point_curr + T.block<3, 1>(0, 3);
    po.x = point_trans.x();
    po.y = point_trans.y();
    po.z = point_trans.z();
    po.intensity = pi.intensity;
}

class PointPlaneFeature
{
public:
    PointPlaneFeature() {}
    Eigen::Vector3d point_;
    Eigen::Vector4d coeffs_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class FeatureExtract
{
public:
    FeatureExtract() {}
   
    template <typename PointType>
    void matchCornerFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_corner_from_map,
                            const typename pcl::PointCloud<PointType> &cloud_map, 
                            const typename pcl::PointCloud<PointType> &cloud_data,
                            const Eigen::Matrix4f &T_local, 
                            std::vector<PointPlaneFeature> &features, 
                            const size_t &N_NEIGH = 5);

    template <typename PointType>
    void matchSurfFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_surf_from_map,
                          const typename pcl::PointCloud<PointType> &cloud_map,
                          const typename pcl::PointCloud<PointType> &cloud_data,
                          const Eigen::Matrix4f &T_local,
                          std::vector<PointPlaneFeature> &features,
                          const size_t &N_NEIGH = 5);
};

template <typename PointType>
void FeatureExtract::matchCornerFromMap(const typename pcl::KdTreeFLANN<PointType>::Ptr &kdtree_corner_from_map,
                                        const typename pcl::PointCloud<PointType> &cloud_map,
                                        const typename pcl::PointCloud<PointType> &cloud_data,
                                        const Eigen::Matrix4f &T_local,
                                        std::vector<PointPlaneFeature> &features,
                                        const size_t &N_NEIGH)
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
        pointAssociateToMap(point_ori, point_sel, T_local);
        kdtree_corner_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
        if (point_search_sq_dis[num_neighbors - 1] < 5.0)
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
                Eigen::Vector3f X0(point_sel.x, point_sel.y, point_sel.z);
                Eigen::Vector3f point_on_line = center;
                Eigen::Vector3f X1, X2;
                X1 = 0.1 * unit_direction + point_on_line;
                X2 = -0.1 * unit_direction + point_on_line;

                Eigen::Vector3f n = (X1 - X0).cross(X2 - X0);
                Eigen::Vector3f w2 = n.normalized();
                Eigen::Vector3f w1 = (w2.cross(X2 - X1)).normalized();
                float ld_1 = n.norm() / (X1 - X2).norm(); // the distance between point to plane
                float ld_2 = 0.0;
                float ld_p1 = -(w1.x() * point_sel.x + w1.y() * point_sel.y + w1.z() * point_sel.z - ld_1);
                float ld_p2 = -(w2.x() * point_sel.x + w2.y() * point_sel.y + w2.z() * point_sel.z - ld_2);
                // float s = 1 - 0.9f * fabs(ld_1);

                Eigen::Vector4d coeff1(w1.x(), w1.y(), w1.z(), ld_p1);
                Eigen::Vector4d coeff2(w2.x(), w2.y(), w2.z(), ld_p2);

                PointPlaneFeature feature1, feature2;

                // feature1.idx_ = i;
                feature1.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                feature1.coeffs_ = coeff1 * 0.5;
                // feature1.laser_idx_ = (size_t)point_ori.intensity;
                // feature1.type_ = 'c';
                features[cloud_cnt] = feature1;
                cloud_cnt++;

                // feature2.idx_ = i;
                feature2.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                feature2.coeffs_ = coeff2 * 0.5;
                // feature2.laser_idx_ = (size_t)point_ori.intensity;
                // feature2.type_ = 'c';
                features[cloud_cnt] = feature2;
                cloud_cnt++;
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
                                      const Eigen::Matrix4f &T_local,
                                      std::vector<PointPlaneFeature> &features,
                                      const size_t &N_NEIGH)
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
        pointAssociateToMap(point_ori, point_sel, T_local);
        kdtree_surf_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
        if (point_search_sq_dis[num_neighbors - 1] < 2.0)
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

            // check if a plane that coeff * [x, y, z, 1] <= 1.0
            bool plane_valid = true;
            for (int j = 0; j < num_neighbors; j++)
            {
                if (fabs(norm(0) * cloud_map.points[point_search_idx[j]].x +
                         norm(1) * cloud_map.points[point_search_idx[j]].y +
                         norm(2) * cloud_map.points[point_search_idx[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    plane_valid = false;
                    break;
                }
            }

            if (plane_valid)
            {
                // pd2 (distance) smaller, s larger
                // float pd2 = norm(0) * point_sel.x + norm(1) * point_sel.y + norm(2) * point_sel.z + negative_OA_dot_norm;
                // float s = 1 - 0.9f * fabs(pd2) / sqrt(sqrSum(point_sel.x, point_sel.y, point_sel.z));
                Eigen::Vector4d coeff(norm(0), norm(1), norm(2), negative_OA_dot_norm);
                PointPlaneFeature feature;

                // feature.idx_ = i;
                feature.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                feature.coeffs_ = coeff;
                // feature.laser_idx_ = (size_t)point_ori.intensity;
                // feature.type_ = 's';
                features[cloud_cnt] = feature;
                cloud_cnt++;
            }
        }
    }
    features.resize(cloud_cnt);
}

#endif
//
