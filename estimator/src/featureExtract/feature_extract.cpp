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

float cloud_curvature[400000];
int cloud_sort_ind[400000];
int cloud_neighbor_picked[400000];
int cloud_label[400000];

bool comp(int i,int j) { return (cloud_curvature[i] < cloud_curvature[j]); }

FeatureExtract::FeatureExtract()
{
    half_passed_ = false;
}

void FeatureExtract::findStartEndAngle(const common::PointCloud &laser_cloud_in)
{
    int cloud_size = laser_cloud_in.points.size();
    start_ori_ = -atan2(laser_cloud_in.points[0].y, laser_cloud_in.points[0].x);
    end_ori_ = -atan2(laser_cloud_in.points[cloud_size - 1].y,
                           laser_cloud_in.points[cloud_size - 1].x) + 2 * M_PI;
    if (end_ori_ - start_ori_ > 3 * M_PI)
    {
        end_ori_ -= 2 * M_PI;
    }
    else if (end_ori_ - start_ori_ < M_PI)
    {
        end_ori_ += 2 * M_PI;
    }
    // std::cout << "end Ori " << end_ori_ << std::endl;
}

void FeatureExtract::cloudRearrange(const common::PointCloud &laser_cloud_in, std::vector<common::PointICloud> &laser_cloud_scans, int &cloud_size)
{
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
            if (ori < start_ori_ - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > start_ori_ + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - start_ori_ > M_PI)
            {
                half_passed_ = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < end_ori_ - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > end_ori_ + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - start_ori_) / (end_ori_ - start_ori_);
        point.intensity = scan_id + SCAN_PERIOD * relTime;
        laser_cloud_scans[scan_id].push_back(point);
    }
    cloud_size = count;
    // printf("points size: %d ********* \n",  cloud_size);
    // for (auto i = 0; i < laser_cloud_scans.size(); i++) printf("line %d: %d\n", i, laser_cloud_scans[i].size());
}

void FeatureExtract::extractCloud(const double &cur_time, const PointCloud &laser_cloud_in, cloudFeature &cloud_feature)
{
    TicToc t_whole, t_prepare;
    std::vector<common::PointICloud> laser_cloud_scans;
    int cloud_size = laser_cloud_in.size();
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

    // step 2: compute curvature of each point
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

    // step 3: 挑选点，排除容易被斜面挡住的点以及离群点，有些点容易被斜面挡住，而离群点可能出现带有偶然性，这些情况都可能导致前后两次扫描不能被同时看到
    // similar to the function of segmentation

    // step 4: extract edge and planar features using curvature
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
        // split the points at each scan into 6 pieces to select features averagely
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
                    if (largest_picked_num <= 2) // select 2 points with maximum curvature
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
                    // remove the neighbor points to make the points distributed at all places
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
                    if (smallest_picked_num >= 4) // select 4 points with minimum curvature
                    {
                        break;
                    }
                    cloud_neighbor_picked[ind] = 1;
                    // remove the neighbor points with large curvature to make the points distributed at all direction
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
    // printf("whole scan registration time %fms \n", t_whole.toc());
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

//
