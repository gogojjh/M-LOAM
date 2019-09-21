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

#include <pcl/common/transforms.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/filters/extract_indices.h>

#include "common/types/type.h"

#include "parameters.h"
#include "../featureExtract/feature_extract.h"
#include "../lidarTracker/lidar_tracker.h"
#include "../initial/initial_extrinsics.h"

#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../utility/CircularBuffer.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/lidar_pivot_point_plane_factor.hpp"
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

    void clearState();
    void setParameter();

    void inputCloud(const double &t, const std::vector<common::PointCloud> &v_laser_cloud_in);
    void inputCloud(const double &t, const common::PointCloud &laser_cloud_in0);

    // interface
    void processMeasurements();
    void process();

    // slide window and marginalization
    void slideWindow();

    // build local map
    void buildLocalMap();

    // process localmap optimization
    void optimizeLocalMap();
    void vector2Double();
    void double2Vector();
    void printParameter();

    void changeSensorType(int use_imu, int use_stereo);


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

    SolverFlag solver_flag_;

    bool b_system_inited_;

    // extrinsic from base to laser
    // std::vector<Pose> pose_ext_;
    // std::vector<Pose> pose_prev_cur_;
    // std::vector<Pose> pose_base_cur_;

    // pose from laser at k=0 to laser at k=K
    std::vector<Pose> pose_laser_cur_;
    // pose from laser at k=K-1 to laser at k=K
    std::vector<Pose> pose_rlt_;

    std::vector<Eigen::Quaterniond> qbl_;
    std::vector<Eigen::Vector3d> tbl_;
    std::vector<double> tdbl_;

    // slide window
    // xx[cir_buf_cnt_] indicates the newest variables and measurements
    bool ini_fixed_local_map_;

    size_t cir_buf_cnt_;

    CircularBuffer<Eigen::Quaterniond> Qs_;
    CircularBuffer<Eigen::Vector3d> Ts_;
    CircularBuffer<std_msgs::Header> Header_;
    std::vector<CircularBuffer<common::PointICloud> > surf_points_stack_, corner_points_stack_;
    std::vector<CircularBuffer<int> > surf_points_stack_size_, corner_points_stack_size_;

    std::vector<common::PointICloud> surf_points_local_map_, surf_points_local_map_filtered_;
    std::vector<common::PointICloud> surf_points_pivot_map_;
    std::vector<common::PointICloud> corner_points_local_map_, corner_points_local_map_filtered_;
    std::vector<common::PointICloud> corner_points_pivot_map_;

    double prev_time_, cur_time_;

    double td_;

    int input_cloud_cnt_;

    FeatureExtract f_extract_;

    LidarTracker lidar_tracker_;

    InitialExtrinsics initial_extrinsics_;

    std::queue<std::pair<double, std::vector<cloudFeature> > > feature_buf_;

    pair<double, std::vector<cloudFeature> > prev_feature_, cur_feature_;

    std::vector<std::vector<std::vector<PointPlaneFeature> > > surf_map_features_;

    double **para_pose_;
    double **para_ex_pose_;
    double *para_td_;

    // for marginalization
    MarginalizationInfo *last_marginalization_info_;
    vector<double *> last_marginalization_parameter_blocks_;
};





//
