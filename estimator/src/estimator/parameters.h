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

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <map>
#include <cassert>
#include <cstdio>

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "common/types/type.h"
#include "common/csvfile.h"
#include "common/color.hpp"
#include "common/timing.hpp"

#include "../estimator/pose.h"

using namespace std;

extern int MLOAM_RESULT_SAVE;
extern std::string OUTPUT_FOLDER;
extern std::string MLOAM_ODOM_PATH;
extern std::string MLOAM_MAP_PATH;
extern std::string MLOAM_GPS_PATH;
extern std::string MLOAM_GT_PATH;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string EX_CALIB_EIG_PATH;

extern int MULTIPLE_THREAD;

extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern int ESTIMATE_EXTRINSIC;
extern int ESTIMATE_TD;

extern int SEGMENT_CLOUD;
extern int HORIZON_SCAN;
extern int MIN_CLUSTER_SIZE;
extern int MIN_LINE_SIZE;
extern int SEGMENT_VALID_POINT_NUM;
extern int SEGMENT_VALID_LINE_NUM;
extern float SEGMENT_THETA;

// LiDAR
extern size_t IDX_REF;
extern size_t NUM_OF_LASER;
extern size_t N_SCANS;

extern int WINDOW_SIZE;
extern int OPT_WINDOW_SIZE;

extern int DISTORTION;
extern float SCAN_PERIOD;
extern float DISTANCE_SQ_THRESHOLD;
extern float NEARBY_SCAN;

extern std::string CLOUD0_TOPIC, CLOUD1_TOPIC;
extern std::vector<std::string> CLOUD_TOPIC;

extern float LASER_SYNC_THRESHOLD;
extern double ROI_RANGE;

extern std::vector<Eigen::Quaterniond> QBL;
extern std::vector<Eigen::Vector3d> TBL;
extern std::vector<double> TDBL;

// odometry
extern int PLANAR_MOVEMENT;

extern float MIN_MATCH_SQ_DIS;
extern float MIN_PLANE_DIS;

extern int MARGINALIZATION_FACTOR;
extern int POINT_PLANE_FACTOR;
extern int POINT_EDGE_FACTOR;
extern int PRIOR_FACTOR;
extern double PRIOR_FACTOR_POS;
extern double PRIOR_FACTOR_ROT;
extern int CHECK_JACOBIAN;

extern int PCL_VIEWER;
extern int PCL_VIEWER_NORMAL_RATIO;

extern int N_CUMU_FEATURE;
extern double LAMBDA_INITIAL;
extern double LAMBDA_THRE_CALIB;
extern int N_CALIB;
extern float ODOM_GF_RATIO;

extern int SKIP_NUM_ODOM_PUB;
extern int LM_OPT_ENABLE;

// mapping
extern float MAP_CORNER_RES;
extern float MAP_SURF_RES;
extern float MAP_OUTLIER_RES;
extern float MAP_SUR_KF_RES;
extern float MAP_EIG_THRE;
extern float MAP_DEG_THRE;

extern float DISTANCE_KEYFRAMES;
extern float ORIENTATION_KEYFRAMES;
extern float SURROUNDING_KF_RADIUS;

extern float UCT_EXT_RATIO;
extern std::vector<Eigen::Matrix<double, 6, 6> > COV_EXT;
extern Eigen::Matrix<double, 3, 3> COV_MEASUREMENT;
extern double TRACE_THRESHOLD_MAPPING;

void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

typedef std::map<std::string, common::PointICloud> cloudFeature;

class PointPlaneFeature
{
public:
    PointPlaneFeature() : idx_(0), laser_idx_(0), type_('n') {}
    size_t idx_;
    size_t laser_idx_;
    Eigen::Vector3d point_;
    Eigen::VectorXd coeffs_;
    Eigen::MatrixXd jaco_;
    char type_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class FeatureWithScore
{
public:
    FeatureWithScore(const int &idx, const double &score, const Eigen::MatrixXd &jaco) 
        : idx_(idx), score_(score), jaco_(jaco) {}

    bool operator < (const FeatureWithScore &fws) const
    {
        return this->score_ < fws.score_;
    }
    
    size_t idx_;
    double score_;
    Eigen::MatrixXd jaco_;
};

class ScanInfo
{
public:
    ScanInfo(const int &n_scan, const bool &segment_flag)
    {
        segment_flag_ = segment_flag;
        scan_start_ind_.resize(n_scan);
        scan_end_ind_.resize(n_scan);
        ground_flag_.clear();
    }

    std::vector<int> scan_start_ind_, scan_end_ind_;
    bool segment_flag_;
    std::vector<bool> ground_flag_;
};