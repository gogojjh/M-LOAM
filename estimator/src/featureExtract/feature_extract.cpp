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

#include "feature_extract.h"

using namespace common;

const double scan_period = 0.1;

float cloud_curvature[400000];
int cloud_sort_ind[400000];
int cloud_neighbor_picked[400000];
int cloud_label[400000];

bool comp (int i,int j) { return (cloud_curvature[i] < cloud_curvature[j]); }

FeatureExtract::FeatureExtract()
{
    half_passed_ = false;
}

void FeatureExtract::cloudRearrange(const PointCloud &laser_cloud_in)
{
    cloud_size_ = laser_cloud_in.points.size();
    float start_ori = -atan2(laser_cloud_in.points[0].y, laser_cloud_in.points[0].x);
    float end_ori = -atan2(laser_cloud_in.points[cloud_size_ - 1].y,
                            laser_cloud_in.points[cloud_size_ - 1].x) +
                            2 * M_PI;
    if (end_ori - start_ori > 3 * M_PI)
    {
        end_ori -= 2 * M_PI;
    }
    else if (end_ori - start_ori < M_PI)
    {
        end_ori += 2 * M_PI;
    }
    // std::cout << "end Ori " << end_ori << std::endl;

    int count = cloud_size_;
    PointI point;
    laser_cloud_scans_.clear();
    laser_cloud_scans_.resize(N_SCANS);
    for (int i = 0; i < cloud_size_; i++)
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
        laser_cloud_scans_[scan_id].push_back(point);
    }
    cloud_size_ = count;
    printf("points size: %d ********* \n",  cloud_size_);
}

cloudFeature FeatureExtract::extractCloud(const double &cur_time, const PointCloud &laser_cloud_in)
{
    t_whole_.tic();
    t_prepare_.tic();

    cloudRearrange(laser_cloud_in);

    std::vector<int> scan_start_ind(N_SCANS);
    std::vector<int> scan_end_ind(N_SCANS);

    PointICloud::Ptr laser_cloud(new PointICloud());
    for (int i = 0; i < N_SCANS; i++)
    {
        scan_start_ind[i] = laser_cloud->size() + 5;
        *laser_cloud += laser_cloud_scans_[i];
        scan_end_ind[i] = laser_cloud->size() - 6;
    }
    printf("prepare time %f ms \n", t_prepare_.toc());

    for (int i = 5; i < cloud_size_ - 5; i++)
    {
        float diff_x = laser_cloud->points[i - 5].x + laser_cloud->points[i - 4].x + laser_cloud->points[i - 3].x + laser_cloud->points[i - 2].x + laser_cloud->points[i - 1].x - 10 * laser_cloud->points[i].x + laser_cloud->points[i + 1].x + laser_cloud->points[i + 2].x + laser_cloud->points[i + 3].x + laser_cloud->points[i + 4].x + laser_cloud->points[i + 5].x;
        float diff_y = laser_cloud->points[i - 5].y + laser_cloud->points[i - 4].y + laser_cloud->points[i - 3].y + laser_cloud->points[i - 2].y + laser_cloud->points[i - 1].y - 10 * laser_cloud->points[i].y + laser_cloud->points[i + 1].y + laser_cloud->points[i + 2].y + laser_cloud->points[i + 3].y + laser_cloud->points[i + 4].y + laser_cloud->points[i + 5].y;
        float diff_z = laser_cloud->points[i - 5].z + laser_cloud->points[i - 4].z + laser_cloud->points[i - 3].z + laser_cloud->points[i - 2].z + laser_cloud->points[i - 1].z - 10 * laser_cloud->points[i].z + laser_cloud->points[i + 1].z + laser_cloud->points[i + 2].z + laser_cloud->points[i + 3].z + laser_cloud->points[i + 4].z + laser_cloud->points[i + 5].z;
        cloud_curvature[i] = sqrSum(diff_x, diff_y, diff_z);
        cloud_sort_ind[i] = i;
        cloud_neighbor_picked[i] = 0;
        cloud_label[i] = 0;
    }

    t_pts_.tic();

    PointICloud corner_points_sharp;
    PointICloud corner_points_less_sharp;
    PointICloud surf_points_flat;
    PointICloud surf_points_less_flat;
    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++)
    {
        if (scan_end_ind[i] - scan_start_ind[i] < 6)
            continue;

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
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloud_sort_ind[k];
                if (cloud_neighbor_picked[ind] == 0 && cloud_curvature[ind] < 0.1)
                {
                    cloud_label[ind] = -1;
                    surf_points_flat.push_back(laser_cloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
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
        PointICloud surf_points_less_flat_scanDS;
        pcl::VoxelGrid<PointI> dosn_size_filter;
        dosn_size_filter.setInputCloud(surf_points_less_flat_scan);
        dosn_size_filter.setLeafSize(0.2, 0.2, 0.2);
        dosn_size_filter.filter(surf_points_less_flat_scanDS);
        surf_points_less_flat += surf_points_less_flat_scanDS;
    }
    printf("sort q time %f ms \n", t_q_sort);
    printf("seperate points time %f ms \n", t_pts_.toc());

    printf("scan registration time %f ms \n", t_whole_.toc());
    if(t_whole_.toc() > 100)
        ROS_WARN("scan registration process over 100ms");

    cloudFeature cloud_feature;
    // cloud_feature.find("key")->first/ second
    // cloud_feature.erase("key")/ cloud_feature.erase(cloud_feature.find("key"))
    cloud_feature.insert(pair<std::string, PointICloud>("laser_cloud", *laser_cloud));
    cloud_feature.insert(pair<std::string, PointICloud>("corner_points_sharp", corner_points_sharp));
    cloud_feature.insert(pair<std::string, PointICloud>("corner_points_less_sharp", corner_points_less_sharp));
    cloud_feature.insert(pair<std::string, PointICloud>("surf_points_flat", surf_points_flat));
    cloud_feature.insert(pair<std::string, PointICloud>("surf_points_less_flat", surf_points_less_flat));
    return cloud_feature;
}




//
