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

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <cmath>
#include <cfloat>
#include <map>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>


#include "common/types/type.h"
#include "common/algos/math.hpp"

#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

class ImageSegmenter
{
public:
    ImageSegmenter()
    {
        // printf("%d, %d, %d, %d\n", min_cluster_size_, min_line_size_, segment_valid_point_num_, segment_valid_line_num_);
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1;
        neighbor.second = 0;
        neighbor_iterator_.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = 1;
        neighbor_iterator_.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = -1;
        neighbor_iterator_.push_back(neighbor);
        neighbor.first = 1;
        neighbor.second = 0;
        neighbor_iterator_.push_back(neighbor);
    }

    void setParameter(const int &vertical_scans,
                      const int &horizon_scans,
                      const int &min_cluster_size,
                      const int &min_line_size,
                      const int &segment_valid_point_num,
                      const int &segment_valid_line_num) 
    {
        vertical_scans_ = vertical_scans;
        horizon_scans_ = horizon_scans;
        min_cluster_size_ = min_cluster_size;
        if (vertical_scans_ == 16)
        {
            ang_res_x_ = 360.0 / horizon_scans_;
            ang_res_y_ = 2.0; 
            ang_bottom_ = 15.0 + 0.1;
            ground_scan_id_ = 7;
        } else
        if (vertical_scans_ == 32)
        {
            ang_res_x_ = 360.0 / horizon_scans_;
            ang_res_y_ = 41.33 / float(vertical_scans_ - 1);
            ang_bottom_ = 30.0 + 0.67;
            ground_scan_id_ = 20;
        }
        // else if (vertical_scans_ == 64)
        // {
        //     std::cout << common::RED << "[ImageSegmenter::setParameter] TBD" << common::RESET << std::endl;
        //     ang_res_y_ = 1 / 0.5;
        //     ang_bottom_ = 15.0 + 0.1;
        // }
        segment_alphax_ = ang_res_x_ / 180.0 * M_PI;
        segment_alphay_ = ang_res_y_ / 180.0 * M_PI;
        min_line_size_ = min_line_size;
        segment_valid_point_num_ = segment_valid_point_num;
        segment_valid_line_num_ = segment_valid_line_num;
    }

    void segmentCloud(const common::PointCloud &laser_cloud_in, common::PointCloud &laser_cloud_out) const;

private:
    int vertical_scans_, horizon_scans_;
    int ground_scan_id_;
    int min_cluster_size_, min_line_size_, segment_valid_point_num_, segment_valid_line_num_;
    float ang_res_x_, ang_res_y_, ang_bottom_;
    float segment_alphax_, segment_alphay_;
    std::vector<pair<int8_t, int8_t> > neighbor_iterator_;
};

//
// LiDAR projection from LeGO-LOAM

// VLP-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 2.0;
// extern const float ang_bottom = 15.0+0.1;
// extern const int groundScanInd = 7;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0 / float(Horizon_SCAN);
// extern const float ang_res_y = 41.33 / float(N_SCAN - 1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet, please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;
