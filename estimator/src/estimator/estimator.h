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

#include <thread>
#include <mutex>
#include <unordered_map>
#include <queue>

#include <omp.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <ceres/ceres.h>

#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/filters/extract_indices.h>

#include "parameters.h"
#include "common/common.hpp"
#include "common/color.hpp"
#include "common/types/type.h"
#include "../imageSegmenter/image_segmenter.hpp"
#include "../featureExtract/feature_extract.hpp"
#include "../lidarTracker/lidar_tracker.h"
#include "../initial/initial_extrinsics.h"
#include "../utility/utility.h"
#include "../utility/cloud_visualizer.h"
#include "../utility/tic_toc.h"
#include "../utility/CircularBuffer.h"
#include "../factor/lidar_pivot_plane_norm_factor.hpp"
#include "../factor/lidar_pivot_target_plane_norm_factor.hpp"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/prior_factor.hpp"

#include "mloam_pcl/point_with_time.hpp"

#define MAX_FEATURE_SELECT_TIME 5 // 7ms
#define MAX_RANDOM_QUEUE_TIME 20

class Estimator
{
  public:
    Estimator();
    ~Estimator();

    void clearState();
    void setParameter();

    void inputCloud(const double &t, const std::vector<common::PointCloud> &v_laser_cloud_in);
    void inputCloud(const double &t, const std::vector<common::PointITimeCloud> &v_laser_cloud_in);
    void inputCloud(const double &t, const common::PointCloud &laser_cloud_in);

    // process measurements
    void processMeasurements();
    void undistortMeasurements(const std::vector<Pose> &pose_undist);
    void process();

    // build global map (for online calibration) and local map (for local optimization)
    void buildCalibMap();
    void buildLocalMap();

    // process localmap optimization
    void optimizeMap();

    // apply good feature
    void evaluateFeatJacobian(const double *para_pose_pivot,
                              const double *para_pose_other,
                              const double *para_pose_ext,
                              const PointPlaneFeature &feature,
                              Eigen::MatrixXd &mat_jaco);

    void evaluateFeatJacobian(const Pose &pose_pivot,
                              const Pose &pose_i,
                              const Pose &pose_ext,
                              const PointPlaneFeature &feature,
                              Eigen::MatrixXd &mat_jaco);

    void goodFeatureSelect(const std::vector<PointPlaneFeature> &all_features,
                           std::vector<size_t> &sel_feature_idx,
                           const double *para_pose_pivot,
                           const double *para_pose_other,
                           const double *para_pose_ext,
                           const double gf_ratio = 0.5);

    void goodFeatureMatching(const pcl::KdTreeFLANN<PointI>::Ptr &kdtree_from_map,
                             const PointICloud &laser_map,
                             const PointICloud &laser_cloud,
                             std::vector<PointPlaneFeature> &all_features,
                             std::vector<size_t> &sel_feature_idx,
                             const char feature_type,
                             const Pose &pose_pivot,
                             const Pose &pose_i,
                             const Pose &pose_ext,
                             const double gf_ratio = 0.5);

    void vector2Double();
    void double2Vector();

    // slide window and marginalization
    void slideWindow();

    void evalResidual(ceres::Problem &problem,
                      std::vector<PoseLocalParameterization *> &local_param_ids);

    void evalDegenracy(std::vector<PoseLocalParameterization *> &local_param_ids,
                       const ceres::CRSMatrix &jaco);

    void evalCalib();

    void printParameter();
    void printSlideWindow();
    void visualizePCL();

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

    omp_lock_t omp_lock_{};

    bool init_thread_flag_;

    SolverFlag solver_flag_;

    bool b_system_inited_{};

    Pose pose_laser_prev_;
    // pose from laser at k=0 to laser at k=K
    std::vector<Pose> pose_laser_cur_;
    // pose from laser at k=K-1 to laser at k=K
    std::vector<Pose> pose_rlt_;

    std::vector<Eigen::Quaterniond> qbl_;
    std::vector<Eigen::Vector3d> tbl_;
    std::vector<double> tdbl_;
    std::vector<Eigen::Matrix<double, 6, 6> > covbl_;

    // slide window
    // xx[cir_buf_cnt_] indicates the newest variables and measurements
    bool ini_fixed_local_map_{};

    size_t cir_buf_cnt_{};

    CircularBuffer<Eigen::Quaterniond> Qs_;
    CircularBuffer<Eigen::Vector3d> Ts_;
    CircularBuffer<std_msgs::Header> Header_;
    std::vector<CircularBuffer<common::PointICloud> > surf_points_stack_, corner_points_stack_;
    std::vector<CircularBuffer<int> > surf_points_stack_size_, corner_points_stack_size_;

    std::vector<common::PointICloud> surf_points_local_map_, surf_points_local_map_filtered_;
    std::vector<common::PointICloud> surf_points_pivot_map_;
    std::vector<common::PointICloud> corner_points_local_map_, corner_points_local_map_filtered_;
    std::vector<common::PointICloud> corner_points_pivot_map_;

    std::vector<std::vector<Pose> > pose_local_;

    double prev_time_{}, cur_time_{};
    double td_{};

    int frame_cnt_{};

    ImageSegmenter img_segment_;
    FeatureExtract f_extract_;
    LidarTracker lidar_tracker_;
    InitialExtrinsics initial_extrinsics_;

    std::queue<std::pair<double, std::vector<cloudFeature> > > feature_buf_;
    pair<double, std::vector<cloudFeature> > prev_feature_, cur_feature_;
    std::vector<std::vector<std::vector<PointPlaneFeature> > > surf_map_features_, corner_map_features_;
    std::vector<std::vector<PointPlaneFeature> > cumu_surf_map_features_, cumu_corner_map_features_;
    size_t cumu_surf_feature_cnt_, cumu_corner_feature_cnt_;

    std::vector<std::vector<std::vector<size_t> > > sel_surf_feature_idx_, sel_corner_feature_idx_;

    double **para_pose_{};
    double **para_ex_pose_{};
    double *para_td_{};

    std::vector<double> eig_thre_;
    std::vector<double> d_factor_calib_;
    std::vector<double> cur_eig_calib_;
    std::vector<std::vector<std::pair<double, Pose> > > pose_calib_;
    std::vector<bool> calib_converge_;

    // for marginalization
    MarginalizationInfo *last_marginalization_info_{};
    vector<double *> last_marginalization_parameter_blocks_;

    PlaneNormalVisualizer plane_normal_vis_;

    std::vector<double> total_measurement_pre_time_, total_feat_matching_time_, 
      total_solver_time_, total_marginalization_time_, total_whole_odom_time_;
    int total_corner_feature_, total_surf_feature_;

    std::vector<nav_msgs::Path> v_laser_path_;

    pcl::PCDWriter pcd_writer_;
};





//
