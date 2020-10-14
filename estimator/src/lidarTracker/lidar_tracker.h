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
#include <map>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <opencv2/opencv.hpp>

#include "common/types/type.h"
#include "common/algos/math.hpp"
#include "../estimator/parameters.h"
#include "../featureExtract/feature_extract.hpp"
#include "../factor/pose_local_parameterization.h"
#include "../factor/lidar_scan_factor.hpp"
#include "../factor/impl_loss_function.hpp"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

class LidarTracker
{
public:
    LidarTracker();
    Pose trackCloud(const cloudFeature &prev_cloud_feature, const cloudFeature &cur_cloud_feature, const Pose &pose_ini);
    void evalDegenracy(PoseLocalParameterization *local_parameterization, const ceres::CRSMatrix &jaco);

    FeatureExtract f_extract_;
};


//
