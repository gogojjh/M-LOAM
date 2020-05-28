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

    void setParameter(const int &horizon_scans,
                      const int &min_cluster_size,
                      const int &min_line_size,
                      const int &segment_valid_point_num,
                      const int &segment_valid_line_num) 
    {
        horizon_scans_ = horizon_scans;
        min_cluster_size_ = min_cluster_size;
        ang_res_x_ = 360.0 / horizon_scans_;
        ang_res_y_ = 2.0; 
        ang_bottom_ = 15.0 + 0.1;
        segment_alphax_ = ang_res_x_ / 180.0 * M_PI;
        segment_alphay_ = ang_res_y_ / 180.0 * M_PI;
        min_line_size_ = min_line_size;
        segment_valid_point_num_ = segment_valid_point_num;
        segment_valid_line_num_ = segment_valid_line_num;
    }

    void segmentCloud(const common::PointCloud &laser_cloud_in, common::PointCloud &laser_cloud_out) const;

private:
    int horizon_scans_;
    int min_cluster_size_, min_line_size_, segment_valid_point_num_, segment_valid_line_num_;
    float ang_res_x_, ang_res_y_, ang_bottom_;
    float segment_alphax_, segment_alphay_;
    std::vector<pair<int8_t, int8_t> > neighbor_iterator_;
};



//
