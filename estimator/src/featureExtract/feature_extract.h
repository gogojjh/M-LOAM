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
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

class FeatureExtract
{
public:
    FeatureExtract();

    void cloudRearrange(const common::PointCloud &laser_cloud_in, std::vector<common::PointICloud> &laser_cloud_scans, int &cloud_size);
    void extractCloud(const double &cur_time, const common::PointCloud &laser_cloud_in, cloudFeature &cloud_feature);

    void extractCornerFromMap(const pcl::KdTreeFLANN<common::PointI>::Ptr &kdtree_corner_from_map,
        const common::PointICloud &cloud_map, const common::PointICloud &cloud_data,
        const Pose &pose_local, std::vector<PointPlaneFeature> &features, const int &N_NEIGH = 5, const bool &CHECK_FOV = true);

    void extractSurfFromMap(const pcl::KdTreeFLANN<common::PointI>::Ptr &kdtree_surf_from_map,
        const common::PointICloud &cloud_map, const common::PointICloud &cloud_data,
        const Pose &pose_local, std::vector<PointPlaneFeature> &features, const int &N_NEIGH = 5, const bool &CHECK_FOV = true);

    bool half_passed_;
    pcl::VoxelGrid<common::PointI> down_size_filter_corner_;
    pcl::VoxelGrid<common::PointI> down_size_filter_surf_;
    pcl::VoxelGrid<common::PointI> down_size_filter_local_map_;
    pcl::VoxelGrid<common::PointI> down_size_filter_global_map_;
};




//
