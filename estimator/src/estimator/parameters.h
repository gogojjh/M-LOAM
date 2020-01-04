/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <map>
#include <cassert>

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "pose.h"
#include "../utility/utility.h"
#include "common/types/type.h"
#include "common/csvfile.h"

using namespace std;

extern std::string OUTPUT_FOLDER;
extern int MLOAM_RESULT_SAVE;
extern std::string MLOAM_ODOM_PATH;
extern std::string MLOAM_MAP_PATH;
extern std::string MLOAM_GT_PATH;
extern std::string EX_CALIB_RESULT_PATH;

extern int MULTIPLE_THREAD;

extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern int ESTIMATE_EXTRINSIC;
extern int ESTIMATE_TD;

// LiDAR
extern int NUM_OF_LASER;
extern int N_SCANS;

extern int IDX_REF;

extern int WINDOW_SIZE;
extern int OPT_WINDOW_SIZE;

extern float SCAN_PERIOD;
extern float DISTANCE_SQ_THRESHOLD;
extern float NEARBY_SCAN;
extern int DISTORTION;

extern int SEGMENT_CLOUD;
extern int HORIZON_SCAN;
extern int MIN_CLUSTER_SIZE;
extern int MIN_LINE_SIZE;
extern int SEGMENT_VALID_POINT_NUM;
extern int SEGMENT_VALID_LINE_NUM;
extern float SEGMENT_THETA;

extern std::string CLOUD0_TOPIC, CLOUD1_TOPIC;
extern float LASER_SYNC_THRESHOLD;
extern double ROI_RANGE;
extern double ROI_RANGE_MAPPING;

extern std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > QBL;
extern std::vector<Eigen::Vector3d> TBL;
extern std::vector<double> TDBL;

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

extern int OPTIMAL_EXTRINSIC;

extern int EVALUATE_RESIDUAL;

extern int PCL_VIEWER;
extern int PCL_VIEWER_NORMAL_RATIO;

extern int OPTIMAL_ODOMETRY;
extern int N_CUMU_FEATURE;

extern double EIG_INITIAL;
extern double EIG_THRE_CALIB;
extern int N_CALIB;

extern Eigen::Matrix<double, 9, 9> XI;
extern double NORM_THRESHOLD;

extern float MAP_CORNER_RES;
extern float MAP_SURF_RES;
extern float MAP_EIG_THRE;

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
    size_t idx_;
    Eigen::Vector3d point_;
    Eigen::Vector4d coeffs_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
