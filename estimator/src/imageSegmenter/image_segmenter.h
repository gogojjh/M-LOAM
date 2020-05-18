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
    ImageSegmenter();

    void setScanParam(const int &horizon_scans, const int &min_cluster_size, const int &min_line_size, const int &segment_valid_point_num, const int &segment_valid_line_num);
    void setParameter();

    void projectCloud(const common::PointCloud &laser_cloud_in);
    void segmentCloud(const common::PointCloud &laser_cloud_in, common::PointCloud &laser_cloud_out);
    void labelGroundPoints();
    void labelComponents(int row, int col);
    void labelConnectLine();

private:
    int horizon_scans_;
    int min_cluster_size_, min_line_size_, segment_valid_point_num_, segment_valid_line_num_;
    float ang_res_x_, ang_res_y_, ang_bottom_;
    float segment_alphax_, segment_alphay_;

    common::PointCloud::Ptr cloud_matrix_;
    cv::Mat range_mat_, label_mat_;
    int label_count_;

    std::vector<pair<int8_t, int8_t> > neighbor_iterator_;

    uint16_t *all_pushed_indx_;
    uint16_t *all_pushed_indy_;

    uint16_t *queue_indx_;
    uint16_t *queue_indy_;

    int *queue_indx_last_negi_;
    int *queue_indy_last_negi_;

    float *queue_last_dis_;
};



//
