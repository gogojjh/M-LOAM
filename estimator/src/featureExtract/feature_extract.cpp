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

void FeatureExtract::extractCloud_aloam(const PointICloud &laserCloudIn,
                                        const ScanInfo &scan_info,
                                        cloudFeature &cloud_feature)
{
    typedef pcl::PointXYZI PointType;

    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    // step 1: cloud_rearrange
    int cloudSize = laserCloudIn.points.size();
    // float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    // float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
    //                       laserCloudIn.points[cloudSize - 1].x) +
    //                2 * M_PI;

    // if (endOri - startOri > 3 * M_PI)
    // {
    //     endOri -= 2 * M_PI;
    // }
    // else if (endOri - startOri < M_PI)
    // {
    //     endOri += 2 * M_PI;
    // }
    //printf("end Ori %f\n", endOri);

    float scanPeriod = 0.1;

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        // float ori = -atan2(point.y, point.x);
        // if (!halfPassed)
        // {
        //     if (ori < startOri - M_PI / 2)
        //     {
        //         ori += 2 * M_PI;
        //     }
        //     else if (ori > startOri + M_PI * 3 / 2)
        //     {
        //         ori -= 2 * M_PI;
        //     }

        //     if (ori - startOri > M_PI)
        //     {
        //         halfPassed = true;
        //     }
        // }
        // else
        // {
        //     ori += 2 * M_PI;
        //     if (ori < endOri - M_PI * 3 / 2)
        //     {
        //         ori += 2 * M_PI;
        //     }
        //     else if (ori > endOri + M_PI / 2)
        //     {
        //         ori -= 2 * M_PI;
        //     }
        // }

        // float relTime = (ori - startOri) / (endOri - startOri);
        // point.intensity = scanID + scanPeriod * relTime;
        point.intensity = scanID + laserCloudIn.points[i].intensity;
        laserCloudScans[scanID].push_back(point);
    }

    cloudSize = count;

    // step 2: cloud_feature_extractor
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
        // std::cout << i << " " << scanStartInd[i] << " " << scanEndInd[i] << std::endl;
    }
    printf("points size %d \n", cloudSize);

    printf("prepare time %f \n", t_prepare.toc());

    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }

    TicToc t_pts;

    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++)
    {
        if (scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            // step 3: extract point feature
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // step 4: extract plane feature
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
                {
                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);
                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);
        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    // printf("sort q time %fms\n", t_q_sort);
    // printf("seperate points time %fms\n", t_pts.toc());
    // printf("whole scan registration time %fms \n", t_whole.toc());
    if (t_whole.toc() > 100)
        ROS_WARN("whole scan registration process over 100ms");

    cloud_feature.clear();
    // cloud_feature.find("key")->first/ second
    // cloud_feature.erase("key")/ cloud_feature.erase(cloud_feature.find("key"))
    cloud_feature.insert(pair<std::string, PointICloud>("laser_cloud", *laserCloud));
    cloud_feature.insert(pair<std::string, PointICloud>("corner_points_sharp", cornerPointsSharp));          // subset: the most distinctive edge points
    cloud_feature.insert(pair<std::string, PointICloud>("corner_points_less_sharp", cornerPointsLessSharp)); // more corner points
    cloud_feature.insert(pair<std::string, PointICloud>("surf_points_flat", surfPointsFlat));                // subset: the most distinctive planar points
    cloud_feature.insert(pair<std::string, PointICloud>("surf_points_less_flat", surfPointsLessFlat));       // more planar points

    std::cout << "feature size: " << laserCloud->size() << " "
              << cornerPointsSharp.size() << " " << cornerPointsLessSharp.size() << " "
              << surfPointsFlat.size() << " " << surfPointsLessFlat.size() << std::endl;

    // pcl::PCDWriter pcd_writer;
    // pcd_writer.write("/tmp/mloam_less_surf.pcd", surfPointsLessFlat);
    // pcd_writer.write("/tmp/mloam_less_edge.pcd", cornerPointsLessSharp);
    // pcd_writer.write("/tmp/mloam_surf.pcd", surfPointsFlat);
    // pcd_writer.write("/tmp/mloam_edge.pcd", cornerPointsSharp);
}

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

    std::cout << "feature size: " << laser_cloud->size() << " " 
              << corner_points_sharp.size() << " " << corner_points_less_sharp.size() << " "
              << surf_points_flat.size() << " " << surf_points_less_flat.size() << std::endl;

    // pcl::PCDWriter pcd_writer;
    // pcd_writer.write("/tmp/mloam_less_surf.pcd", surf_points_less_flat);
    // pcd_writer.write("/tmp/mloam_less_edge.pcd", corner_points_less_sharp);
    // pcd_writer.write("/tmp/mloam_surf.pcd", surf_points_flat);
    // pcd_writer.write("/tmp/mloam_edge.pcd", corner_points_sharp);
    
}

//
