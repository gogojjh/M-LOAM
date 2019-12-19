/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

// tutorial about LOAM: https://zhuanlan.zhihu.com/p/57351961

#include "feature_extract.h"

using namespace common;

const double scan_period = 0.1;

float cloud_curvature[400000];
int cloud_sort_ind[400000];
int cloud_neighbor_picked[400000];
int cloud_label[400000];

bool comp(int i,int j) { return (cloud_curvature[i] < cloud_curvature[j]); }

FeatureExtract::FeatureExtract()
{
    half_passed_ = false;
    down_size_filter_corner_.setLeafSize(0.2, 0.2, 0.2);
    down_size_filter_surf_.setLeafSize(0.4, 0.4, 0.4);
    down_size_filter_local_map_.setLeafSize(0.4, 0.4, 0.4);
    down_size_filter_global_map_.setLeafSize(0.6, 0.6, 0.6);
}

void FeatureExtract::cloudRearrange(const common::PointCloud &laser_cloud_in, std::vector<common::PointICloud> &laser_cloud_scans, int &cloud_size)
{
    cloud_size = laser_cloud_in.points.size();
    float start_ori = -atan2(laser_cloud_in.points[0].y, laser_cloud_in.points[0].x);
    float end_ori = -atan2(laser_cloud_in.points[cloud_size - 1].y,
                            laser_cloud_in.points[cloud_size - 1].x) + 2 * M_PI;
    if (end_ori - start_ori > 3 * M_PI)
    {
        end_ori -= 2 * M_PI;
    }
    else if (end_ori - start_ori < M_PI)
    {
        end_ori += 2 * M_PI;
    }
    // std::cout << "end Ori " << end_ori << std::endl;

    int count = cloud_size;
    PointI point;
    laser_cloud_scans.clear();
    laser_cloud_scans.resize(N_SCANS);
    for (int i = 0; i < cloud_size; i++)
    {
        point.x = laser_cloud_in.points[i].x;
        point.y = laser_cloud_in.points[i].y;
        point.z = laser_cloud_in.points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scan_id = 0;

        if (N_SCANS == 16)
        {
            scan_id = int((angle + 15) / 2 + 0.5);
            if (scan_id > (N_SCANS - 1) || scan_id < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scan_id = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scan_id > (N_SCANS - 1) || scan_id < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {
            if (angle >= -8.83)
                scan_id = int((2 - angle) * 3.0 + 0.5);
            else
                scan_id = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            if (angle > 2 || angle < -24.33 || scan_id > 50 || scan_id < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            std::cout << "wrong scan number" << std::endl;
            ROS_BREAK();
        }
        // std::cout << "angle " << angle << ", scan_id " << scan_id << std::endl;

        float ori = -atan2(point.y, point.x);
        if (!half_passed_)
        {
            if (ori < start_ori - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > start_ori + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - start_ori > M_PI)
            {
                half_passed_ = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < end_ori - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > end_ori + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - start_ori) / (end_ori - start_ori);
        point.intensity = scan_id + scan_period * relTime;
        laser_cloud_scans[scan_id].push_back(point);
    }
    cloud_size = count;
    // printf("points size: %d ********* \n",  cloud_size);
    // for (auto i = 0; i < laser_cloud_scans.size(); i++) printf("line %d: %d\n", i, laser_cloud_scans[i].size());
}

void FeatureExtract::extractCloud(const double &cur_time, const PointCloud &laser_cloud_in, cloudFeature &cloud_feature)
{
    TicToc t_whole, t_prepare;
    int cloud_size;
    std::vector<common::PointICloud> laser_cloud_scans;
    cloudRearrange(laser_cloud_in, laser_cloud_scans, cloud_size);

    std::vector<int> scan_start_ind(N_SCANS);
    std::vector<int> scan_end_ind(N_SCANS);

    PointICloud::Ptr laser_cloud(new PointICloud());
    for (int i = 0; i < N_SCANS; i++)
    {
        scan_start_ind[i] = laser_cloud->size() + 5;
        *laser_cloud += laser_cloud_scans[i];
        scan_end_ind[i] = laser_cloud->size() - 6;
    }
    // printf("prepare time %fms\n", t_prepare.toc());

    // compute curvature of each point
    for (int i = 5; i < cloud_size - 5; i++)
    {
        float diff_x = laser_cloud->points[i - 5].x + laser_cloud->points[i - 4].x + laser_cloud->points[i - 3].x + laser_cloud->points[i - 2].x + laser_cloud->points[i - 1].x - 10 * laser_cloud->points[i].x + laser_cloud->points[i + 1].x + laser_cloud->points[i + 2].x + laser_cloud->points[i + 3].x + laser_cloud->points[i + 4].x + laser_cloud->points[i + 5].x;
        float diff_y = laser_cloud->points[i - 5].y + laser_cloud->points[i - 4].y + laser_cloud->points[i - 3].y + laser_cloud->points[i - 2].y + laser_cloud->points[i - 1].y - 10 * laser_cloud->points[i].y + laser_cloud->points[i + 1].y + laser_cloud->points[i + 2].y + laser_cloud->points[i + 3].y + laser_cloud->points[i + 4].y + laser_cloud->points[i + 5].y;
        float diff_z = laser_cloud->points[i - 5].z + laser_cloud->points[i - 4].z + laser_cloud->points[i - 3].z + laser_cloud->points[i - 2].z + laser_cloud->points[i - 1].z - 10 * laser_cloud->points[i].z + laser_cloud->points[i + 1].z + laser_cloud->points[i + 2].z + laser_cloud->points[i + 3].z + laser_cloud->points[i + 4].z + laser_cloud->points[i + 5].z;
        cloud_curvature[i] = sqrSum(diff_x, diff_y, diff_z);
        cloud_sort_ind[i] = i;
        cloud_neighbor_picked[i] = 0;
        cloud_label[i] = 0;
    }

    // extract features using curvature
    TicToc t_pts;
    PointICloud corner_points_sharp;
    PointICloud corner_points_less_sharp;
    PointICloud surf_points_flat;
    PointICloud surf_points_less_flat;
    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++)
    {
        if (scan_end_ind[i] - scan_start_ind[i] < 6) continue;
        PointICloud::Ptr surf_points_less_flat_scan(new PointICloud);
        for (int j = 0; j < 6; j++)
        {
            // step 3: extract point feature
            int sp = scan_start_ind[i] + (scan_end_ind[i] - scan_start_ind[i]) * j / 6;
            int ep = scan_start_ind[i] + (scan_end_ind[i] - scan_start_ind[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            std::sort (cloud_sort_ind + sp, cloud_sort_ind + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            int largest_picked_num = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloud_sort_ind[k];
                if (cloud_neighbor_picked[ind] == 0 && cloud_curvature[ind] > 0.1)
                {
                    largest_picked_num++;
                    if (largest_picked_num <= 2)
                    {
                        cloud_label[ind] = 2;
                        corner_points_sharp.push_back(laser_cloud->points[ind]);
                        corner_points_less_sharp.push_back(laser_cloud->points[ind]);
                    }
                    else if (largest_picked_num <= 20)
                    {
                        cloud_label[ind] = 1;
                        corner_points_less_sharp.push_back(laser_cloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }
                    cloud_neighbor_picked[ind] = 1;
                    // TODO: check extreme cases that object is occlused
                    for (int l = 1; l <= 5; l++)
                    {
                        float diff_x = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l - 1].x;
                        float diff_y = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l - 1].y;
                        float diff_z = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l - 1].z;
                        if (sqrSum(diff_x, diff_y, diff_z) > 0.05)
                        {
                            break;
                        }
                        cloud_neighbor_picked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diff_x = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l + 1].x;
                        float diff_y = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l + 1].y;
                        float diff_z = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l + 1].z;
                        if (sqrSum(diff_x, diff_y, diff_z) > 0.05)
                        {
                            break;
                        }
                        cloud_neighbor_picked[ind + l] = 1;
                    }
                }
            }

            // step 4: extract plane feature
            int smallest_picked_num = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloud_sort_ind[k];
                if (cloud_neighbor_picked[ind] == 0 && cloud_curvature[ind] < 0.1)
                {
                    cloud_label[ind] = -1;
                    surf_points_flat.push_back(laser_cloud->points[ind]);
                    smallest_picked_num++;
                    if (smallest_picked_num >= 4)
                    {
                        break;
                    }
                    cloud_neighbor_picked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diff_x = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l - 1].x;
                        float diff_y = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l - 1].y;
                        float diff_z = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l - 1].z;
                        if (sqrSum(diff_x, diff_y, diff_z) > 0.05)
                        {
                            break;
                        }
                        cloud_neighbor_picked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diff_x = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l + 1].x;
                        float diff_y = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l + 1].y;
                        float diff_z = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l + 1].z;
                        if (sqrSum(diff_x, diff_y, diff_z) > 0.05)
                        {
                            break;
                        }
                        cloud_neighbor_picked[ind + l] = 1;
                    }
                }
            }
            for (int k = sp; k <= ep; k++)
            {
                if (cloud_label[k] <= 0)
                {
                    surf_points_less_flat_scan->push_back(laser_cloud->points[k]);
                }
            }
        }
        PointICloud surf_points_less_flat_scan_ds;
        pcl::VoxelGrid<PointI> down_size_filter;
        down_size_filter.setInputCloud(surf_points_less_flat_scan);
        down_size_filter.setLeafSize(0.2, 0.2, 0.2);
        down_size_filter.filter(surf_points_less_flat_scan_ds);
        surf_points_less_flat += surf_points_less_flat_scan_ds;
    }
    // printf("sort q time %fms\n", t_q_sort);
    // printf("seperate points time %fms\n", t_pts.toc());

    printf("whole scan registration time %fms \n", t_whole.toc());
    if(t_whole.toc() > 100) ROS_WARN("whole scan registration process over 100ms");

    cloud_feature.clear();
    // cloud_feature.find("key")->first/ second
    // cloud_feature.erase("key")/ cloud_feature.erase(cloud_feature.find("key"))
    cloud_feature.insert(pair<std::string, PointICloud>("laser_cloud", *laser_cloud));
    cloud_feature.insert(pair<std::string, PointICloud>("corner_points_sharp", corner_points_sharp));
    cloud_feature.insert(pair<std::string, PointICloud>("corner_points_less_sharp", corner_points_less_sharp));
    cloud_feature.insert(pair<std::string, PointICloud>("surf_points_flat", surf_points_flat));
    cloud_feature.insert(pair<std::string, PointICloud>("surf_points_less_flat", surf_points_less_flat));
}

void FeatureExtract::extractCornerFromMap(const pcl::KdTreeFLANN<PointI>::Ptr &kdtree_corner_from_map,
    const PointICloud &cloud_map,
    const PointICloud &cloud_data,
    const Pose &pose_local,
    std::vector<PointPlaneFeature> &features,
    const int &N_NEIGH,
    const bool &CHECK_FOV)
{
    features.clear();
    std::vector<int> point_search_idx(N_NEIGH, 0);
    std::vector<float> point_search_sq_dis(N_NEIGH, 0);

    // extract edge coefficients and correspondences from edge map
    size_t cloud_size = cloud_data.points.size();
    features.resize(cloud_size * 2);
    size_t cloud_cnt = 0;
    for (size_t i = 0; i < cloud_size; i++)
    {
        PointI point_ori = cloud_data.points[i];
        PointI point_sel;
        pointAssociateToMap(point_ori, point_sel, pose_local);
        int num_neighbors = N_NEIGH;
        kdtree_corner_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
        if (point_search_sq_dis[num_neighbors - 1] < MIN_MATCH_SQ_DIS)
        {
            // calculate the coefficients of edge points
            std::vector<Eigen::Vector3d> near_corners;
            Eigen::Vector3d center(0, 0, 0); // mean value
            for (int j = 0; j < num_neighbors; j++)
            {
                Eigen::Vector3d tmp(cloud_map.points[point_search_idx[j]].x,
                                    cloud_map.points[point_search_idx[j]].y,
                                    cloud_map.points[point_search_idx[j]].z);
                center += tmp;
                near_corners.push_back(tmp);
            }
            center /= (1.0 * num_neighbors);
            Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < num_neighbors; j++)
            {
                Eigen::Vector3d tmp_zero_mean = near_corners[j] - center;
                cov_mat += tmp_zero_mean * tmp_zero_mean.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> esolver(cov_mat);

            // if is indeed line feature
            // note Eigen library sort eigenvalues in increasing order
            Eigen::Vector3d unit_direction = esolver.eigenvectors().col(2);
            if (esolver.eigenvalues()[2] > 3 * esolver.eigenvalues()[1])
            {
                Eigen::Vector3d X0(point_sel.x, point_sel.y, point_sel.z);
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d X1, X2;
                X1 = 0.1 * unit_direction + point_on_line;
                X2 = -0.1 * unit_direction + point_on_line;

                Eigen::Vector3d a012_vec = (X0 - X1).cross(X0 - X2);
                double a012 = a012_vec.norm();
                Eigen::Vector3d w1 = ((X1 - X2).cross(a012_vec)).normalized(); // w1
                Eigen::Vector3d w2 = (X1 - X2).cross(w1); // w2
                double l12 = (X1 - X2).norm();
                double la = w1.x();
                double lb = w1.y();
                double lc = w1.z();
                double ld2 = a012 / l12; // distance

                PointI point_proj = point_sel;
                point_proj.x -= la * ld2;
                point_proj.y -= lb * ld2;
                point_proj.z -= lc * ld2;
                double ld_p2 = -(w2.x() * point_proj.x + w2.y() * point_proj.y + w2.z() * point_proj.z);
                double ld_p1 = -(w1.x() * point_proj.x + w1.y() * point_proj.y + w1.z() * point_proj.z);
                double s = 1 - 0.9f * fabs(ld2);
                Eigen::Vector4d coeff1(la, lb, lc, ld_p1);
                Eigen::Vector4d coeff2(w2.x(), w2.y(), w2.z(), ld_p2);
                bool is_in_laser_fov = false;
                if (CHECK_FOV)
                {
                    PointI point_on_z_axis, point_on_z_axis_trans;
                    point_on_z_axis.x = 0.0;
                    point_on_z_axis.y = 0.0;
                    point_on_z_axis.z = 10.0;
                    pointAssociateToMap(point_on_z_axis, point_on_z_axis_trans, pose_local);
                    double squared_side1 = sqrSum(pose_local.t_(0) - point_sel.x,
                                                 pose_local.t_(1) - point_sel.y,
                                                 pose_local.t_(2) - point_sel.z);
                    double squared_side2 = sqrSum(point_on_z_axis_trans.x - point_sel.x,
                                                 point_on_z_axis_trans.y - point_sel.y,
                                                 point_on_z_axis_trans.z - point_sel.z);

                    double check1 = 100.0f + squared_side1 - squared_side2 - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                    double check2 = 100.0f + squared_side1 - squared_side2 + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                    // within +-60 degree
                    if (check1 < 0 && check2 > 0) is_in_laser_fov = true;
                } else
                {
                    is_in_laser_fov = true;
                }
                if (s > 0.1 && is_in_laser_fov)
                {
                    PointPlaneFeature feature1, feature2;
                    feature1.score_ = s * 0.5;
                    feature1.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                    feature1.coeffs_ = coeff1;
                    features[cloud_cnt] = feature1;
                    cloud_cnt++;

                    feature2.score_ = s * 0.5;
                    feature2.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                    feature2.coeffs_ = coeff2;
                    features[cloud_cnt] = feature2;
                    cloud_cnt++;
                }
            }
        }
    }
    features.resize(cloud_cnt);
}

// should be performed once after several gradient descents
void FeatureExtract::extractSurfFromMap(const pcl::KdTreeFLANN<PointI>::Ptr &kdtree_surf_from_map,
    const PointICloud &cloud_map,
    const PointICloud &cloud_data,
    const Pose &pose_local,
    std::vector<PointPlaneFeature> &features,
    const int &N_NEIGH,
    const bool &CHECK_FOV)
{
    features.clear();

    // setting variables
    std::vector<int> point_search_idx(N_NEIGH, 0);
    std::vector<float> point_search_sq_dis(N_NEIGH, 0);
    Eigen::MatrixXd mat_A = Eigen::MatrixXd::Zero(N_NEIGH, 3);
    Eigen::MatrixXd mat_B = Eigen::MatrixXd::Constant(N_NEIGH, 1, -1);

    size_t cloud_size = cloud_data.points.size();
    features.resize(cloud_size);
    size_t cloud_cnt = 0;
    for (size_t i = 0; i < cloud_size; i++)
    {
        PointI point_ori = cloud_data.points[i];
        PointI point_sel;
        pointAssociateToMap(point_ori, point_sel, pose_local);
        int num_neighbors = N_NEIGH;
        kdtree_surf_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
        if (point_search_sq_dis[num_neighbors - 1] < MIN_MATCH_SQ_DIS)
        {
            // PCA eigen decomposition to fit a plane
            for (int j = 0; j < num_neighbors; j++)
            {
                mat_A(j, 0) = cloud_map.points[point_search_idx[j]].x;
                mat_A(j, 1) = cloud_map.points[point_search_idx[j]].y;
                mat_A(j, 2) = cloud_map.points[point_search_idx[j]].z;
            }
            Eigen::Vector3d norm = mat_A.colPivHouseholderQr().solve(mat_B);
            double negative_OA_dot_norm = 1 / norm.norm();
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
                // pd2 smaller, s larger
                double pd2 = norm(0) * point_sel.x + norm(1) * point_sel.y + norm(2) * point_sel.z + negative_OA_dot_norm;
                double s = 1 - 0.9f * fabs(pd2) / sqrt(sqrSum(point_sel.x, point_sel.y, point_sel.z));
                Eigen::Vector4d coeff(norm(0), norm(1), norm(2), negative_OA_dot_norm);
                bool is_in_laser_fov = false;
                if (CHECK_FOV)
                {
                    PointI transform_pos;
                    PointI point_on_z_axis, point_on_z_axis_trans;
                    point_on_z_axis.x = 0.0;
                    point_on_z_axis.y = 0.0;
                    point_on_z_axis.z = 10.0;
                    pointAssociateToMap(point_on_z_axis, point_on_z_axis_trans, pose_local);
                    double squared_side1 = sqrSum(pose_local.t_(0) - point_sel.x,
                                                 pose_local.t_(1) - point_sel.y,
                                                 pose_local.t_(2) - point_sel.z);
                    double squared_side2 = sqrSum(point_on_z_axis_trans.x - point_sel.x,
                                                 point_on_z_axis_trans.y - point_sel.y,
                                                 point_on_z_axis_trans.z - point_sel.z);
                    double check1 = 100.0f + squared_side1 - squared_side2 - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                    double check2 = 100.0f + squared_side1 - squared_side2 + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
                    // within +-60 degree
                    if (check1 < 0 && check2 > 0) is_in_laser_fov = true;
                } else
                {
                    is_in_laser_fov = true;
                }
                if (s > 0.1 && is_in_laser_fov)
                {
                    PointPlaneFeature feature;
                    feature.score_ = s;
                    feature.point_ = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
                    feature.coeffs_ = coeff;
                    features[cloud_cnt] = feature;
                    cloud_cnt++;
                }
            }
        }
    }
    features.resize(cloud_cnt);
}






//
