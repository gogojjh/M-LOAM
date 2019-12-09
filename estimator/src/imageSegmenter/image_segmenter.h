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

#include "common/types/type.h"
#include "common/algos/math.hpp"

#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

class ImageSegmenter
{
public:
    ImageSegmenter();

    void setScanParam(const int &horizon_scans, const int &min_cluster_size);
    void setParameter();

    void projectCloud(const common::PointCloud &laser_cloud_in);
    void segmentCloud(const common::PointCloud &laser_cloud_in, common::PointCloud &laser_cloud_out);
    void labelComponents(int row, int col);

private:
    int horizon_scans_, min_cluster_size_;
    float ang_res_x_, ang_res_y_, ang_bottom_;
    float segment_alphax_, segment_alphay_;

    common::PointCloud::Ptr cloud_matrix_;
    cv::Mat range_mat_, label_mat_;
    int label_count_;

    std::vector<pair<int8_t, int8_t> > neighbor_iterator_;

    uint16_t *all_pushed_indx;
    uint16_t *all_pushed_indy;

    uint16_t *queue_indx;
    uint16_t *queue_indy;
};



//
