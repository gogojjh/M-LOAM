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

#ifndef LOOP_REGISTRATION_HPP
#define LOOP_REGISTRATION_HPP

#include <iostream>
#include <string>

#include <ceres/ceres.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "parameters.hpp"
#include "utility/tic_toc.h"
#include "utility/feature_extract.hpp"
#include "factor/pose_local_parameterization.h"
#include "factor/lidar_map_plane_norm_factor.hpp"
#include "factor/impl_loss_function.hpp"
#include "../ThirdParty/FastGlobalRegistration/app.h"

class LoopRegistration
{
public:

    LoopRegistration() {}

    void parseFPFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                   const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfh_feature,
                   fgr::Points &points,
                   fgr::Feature &features);

    std::pair<bool, Eigen::Matrix4d> performGlobalRegistration(pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_map,
                                                               pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud);

    std::pair<bool, Eigen::Matrix4d> performLocalRegistration(const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf_from_map,
                                                              const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner_from_map,
                                                              const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf,
                                                              const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner,
                                                              const Eigen::Matrix4d &T_ini);

    FeatureExtract f_extract_;
};

#endif
//
