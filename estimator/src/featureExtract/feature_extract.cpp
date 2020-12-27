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

// tutorial about LOAM: https://zhuanlan.zhihu.com/p/57351961

#include "feature_extract.hpp"

using namespace common;

float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

void FeatureExtract::findStartEndTime(const PointITimeCloud &laser_cloud_in,
                                      float &start_time,
                                      float &end_time)
{
    start_time = 1e6;
    end_time = 0.0;
    for (const auto &point : laser_cloud_in)
    {
        start_time = std::min(point.timestamp, start_time);
        end_time = std::max(point.timestamp, end_time);
    }
}

void FeatureExtract::calTimestamp(const PointITimeCloud &laser_cloud_in,
                                  PointICloud &laser_cloud_out)
{
    pcl::copyPointCloud(laser_cloud_in, laser_cloud_out);
    for (size_t i = 0; i < laser_cloud_out.size(); i++)
    {
        // point.x = laser_cloud_in.points[i].x;
        // point.y = laser_cloud_in.points[i].y;
        // point.z = laser_cloud_in.points[i].z;
        // float rel_time = laser_cloud_in.points[i].timestamp * 1e-6;
        // point.intensity = rel_time;
        // laser_cloud_out.push_back(point);
        laser_cloud_out.points[i].intensity = laser_cloud_in.points[i].timestamp * 1e-6;
    }
}

void FeatureExtract::findStartEndAngle(const PointCloud &laser_cloud_in,
                                       float &start_ori,
                                       float &end_ori)
{
    start_ori = -atan2(laser_cloud_in.points[0].y, laser_cloud_in.points[0].x);
    end_ori = -atan2(laser_cloud_in.points[laser_cloud_in.points.size() - 1].y,
                     laser_cloud_in.points[laser_cloud_in.points.size() - 1].x) +
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
}

void FeatureExtract::calTimestamp(const PointCloud &laser_cloud_in,
                                  PointICloud &laser_cloud_out)
{
    float start_ori, end_ori;
    findStartEndAngle(laser_cloud_in, start_ori, end_ori);

    pcl::copyPointCloud(laser_cloud_in, laser_cloud_out);
    bool half_passed = false;
    for (size_t i = 0; i < laser_cloud_out.size(); i++)
    {
        float ori = -atan2(laser_cloud_out.points[i].y, laser_cloud_out.points[i].x);
        if (!half_passed)
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
                half_passed = true;
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
        float rel_time = (ori - start_ori) / (end_ori - start_ori) * SCAN_PERIOD;
        laser_cloud_out.points[i].intensity = rel_time;
    }
}

bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

void FeatureExtract::extractCloud(const PointICloud &laser_cloud_in,
                                  const ScanInfo &scan_info,
                                  cloudFeature &cloud_feature)
{
    TicToc t_whole;

    // compute curvature of each point
    const PointICloud::Ptr laser_cloud = boost::make_shared<PointICloud>(laser_cloud_in);
    size_t cloud_size = laser_cloud->size();
    // printf("points size %d\n", cloud_size);

    float cloud_curvature[cloud_size];
    int cloud_sort_ind[cloud_size];
    int cloud_neighbor_picked[cloud_size];
    int cloud_label[cloud_size];
    for (size_t i = 5; i < cloud_size - 5; i++)
    {
        float diff_x = laser_cloud->points[i - 5].x + laser_cloud->points[i - 4].x + laser_cloud->points[i - 3].x + laser_cloud->points[i - 2].x + laser_cloud->points[i - 1].x - 10 * laser_cloud->points[i].x + laser_cloud->points[i + 1].x + laser_cloud->points[i + 2].x + laser_cloud->points[i + 3].x + laser_cloud->points[i + 4].x + laser_cloud->points[i + 5].x;
        float diff_y = laser_cloud->points[i - 5].y + laser_cloud->points[i - 4].y + laser_cloud->points[i - 3].y + laser_cloud->points[i - 2].y + laser_cloud->points[i - 1].y - 10 * laser_cloud->points[i].y + laser_cloud->points[i + 1].y + laser_cloud->points[i + 2].y + laser_cloud->points[i + 3].y + laser_cloud->points[i + 4].y + laser_cloud->points[i + 5].y;
        float diff_z = laser_cloud->points[i - 5].z + laser_cloud->points[i - 4].z + laser_cloud->points[i - 3].z + laser_cloud->points[i - 2].z + laser_cloud->points[i - 1].z - 10 * laser_cloud->points[i].z + laser_cloud->points[i + 1].z + laser_cloud->points[i + 2].z + laser_cloud->points[i + 3].z + laser_cloud->points[i + 4].z + laser_cloud->points[i + 5].z;
        cloud_curvature[i] = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
        cloud_sort_ind[i] = i;
        cloud_neighbor_picked[i] = 0;
        cloud_label[i] = 0;
    }

    // extract edge and planar features using curvature
    // TicToc t_pts;
    PointICloud corner_points_sharp;
    PointICloud corner_points_less_sharp;
    PointICloud surf_points_flat;
    PointICloud surf_points_less_flat;
    compObject comp_object;
    comp_object.cloud_curvature = cloud_curvature;
    for (size_t i = 0; i < N_SCANS; i++)
    {
        // printf("extract feature, scans: %lu\n", i);
        if (scan_info.scan_end_ind_[i] - scan_info.scan_start_ind_[i] < 6) continue;
        PointICloud::Ptr surf_points_less_flat_scan(new PointICloud);
        // split the points at each scan into 6 pieces to select features averagely
        for (int j = 0; j < 6; j++)
        {
            int sp = scan_info.scan_start_ind_[i] + (scan_info.scan_end_ind_[i] - scan_info.scan_start_ind_[i]) * j / 6;
            int ep = scan_info.scan_start_ind_[i] + (scan_info.scan_end_ind_[i] - scan_info.scan_start_ind_[i]) * (j + 1) / 6 - 1;
            std::sort(cloud_sort_ind + sp, cloud_sort_ind + ep + 1, comp_object); // sort from smallest to largest

            // extract edge feature
            int largest_picked_num = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloud_sort_ind[k];
                // if (cloud_neighbor_picked[ind] == 0 && cloud_curvature[ind] > 0.1
                //                                     && !scan_info.ground_flag_[ind])
                if (cloud_neighbor_picked[ind] == 0 && cloud_curvature[ind] > 0.1)
                {
                    largest_picked_num++;
                    if (largest_picked_num <= 2) // select if and only if existing 2 points with maximum curvature
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
                        if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
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
                        if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
                        {
                            break;
                        }
                        cloud_neighbor_picked[ind + l] = 1;
                    }
                }
            }

            // extract plane feature
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
                        if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
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
                        if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
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
    // printf("seperate points time %fms\n", t_pts.toc());
    // printf("whole scan registration time %fms \n", t_whole.toc());
    if (t_whole.toc() > 100)
        ROS_WARN("whole scan registration process over 100ms");

    cloud_feature.clear();
    // cloud_feature.find("key")->first/ second
    // cloud_feature.erase("key")/ cloud_feature.erase(cloud_feature.find("key"))
    cloud_feature.insert(pair<std::string, PointICloud>("laser_cloud", *laser_cloud));
    cloud_feature.insert(pair<std::string, PointICloud>("corner_points_sharp", corner_points_sharp)); // subset: the most distinctive edge points
    cloud_feature.insert(pair<std::string, PointICloud>("corner_points_less_sharp", corner_points_less_sharp)); // more corner points
    cloud_feature.insert(pair<std::string, PointICloud>("surf_points_flat", surf_points_flat)); // subset: the most distinctive planar points
    cloud_feature.insert(pair<std::string, PointICloud>("surf_points_less_flat", surf_points_less_flat)); // more planar points

    // std::cout << "feature size: " << laser_cloud->size() << " " 
    //           << corner_points_sharp.size() << " " << corner_points_less_sharp.size() << " "
    //           << surf_points_flat.size() << " " << surf_points_less_flat.size() << std::endl;

    // pcl::PCDWriter pcd_writer;
    // pcd_writer.write("/tmp/mloam_less_surf.pcd", surf_points_less_flat);
    // pcd_writer.write("/tmp/mloam_less_edge.pcd", corner_points_less_sharp);
    // pcd_writer.write("/tmp/mloam_surf.pcd", surf_points_flat);
    // pcd_writer.write("/tmp/mloam_edge.pcd", corner_points_sharp);
    
}

//
