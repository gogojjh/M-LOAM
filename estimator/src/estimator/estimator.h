/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "common/types/type.h"

#include "parameters.h"
#include "../featureExtract/feature_extract.h"
#include "../lidarTracker/lidar_tracker.h"
#include "../initial/initial_extrinsics.h"

#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"

class Estimator
{
  public:
    Estimator();
    ~Estimator();

    void inputCloud(const double &t, const std::vector<common::PointCloud> &v_laser_cloud_in);
    void inputCloud(const double &t, const common::PointCloud &laser_cloud_in0);

    void setParameter();

    // interface
    void processMeasurements();
    void changeSensorType(int use_imu, int use_stereo);

    // internal
    void clearState();

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    std::mutex m_process_;
    std::mutex m_buf_;

    std::thread track_thread_;
    std::thread process_thread_;

    bool init_thread_flag_;

    int frame_cnt_;

    SolverFlag solver_flag_;

    bool b_system_inited_;

    // extrinsic from base to laser
    std::vector<Pose> calib_base_laser_;
    
    // pose from laser at k=0 to laser at k=K
    std::vector<std::vector<Pose> > pose_laser_cur_;
    // pose from laser at k=K-1 to laser at k=K
    std::vector<std::vector<Pose> > pose_prev_cur_;
    // std::vector<Pose> pose_ext_;

    double prev_time_, cur_time_;

    double td_;

    int input_cloud_cnt_;

    FeatureExtract f_extract_;

    LidarTracker lidar_tracker_;

    InitialExtrinsics initial_extrinsics_;

    std::queue<std::pair<double, std::vector<cloudFeature> > > feature_buf_;

    pair<double, std::vector<cloudFeature> > prev_feature_, cur_feature_;

};





//
