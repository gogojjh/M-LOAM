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

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <cmath>
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
#include "../featureExtract/feature_extract.h"
#include "../utility/tic_toc.h"

int corner_correspondence = 0;
int plane_correspondence = 0;
int skip_frame_num = 5;

class lidarTracker
{
public:
    LidarTracker();

    void TransformToStart(common::PointI const *const pi, common::PointI *const po);
    void TransformToEnd(common::PointI const *const pi, common::PointI *const po);

    void trackCloud(const cloudFeature &prev_cloud_feature, const cloudFeature &cur_cloud_feature);

    bool b_track_inited_;

    Eigen::Quaterniond q_;
    Eigen::Vector3d t_;
};


//
