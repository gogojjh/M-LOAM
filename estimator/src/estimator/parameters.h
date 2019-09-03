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

const double FOCAL_LENGTH = 460.0;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string OUTPUT_FOLDER;
extern std::string IMU_TOPIC;
extern double TD;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern int ROW, COL;
extern int NUM_OF_CAM;
extern int STEREO;
extern int USE_IMU;
extern int MULTIPLE_THREAD;
// pts_gt for debug purpose;
extern map<int, Eigen::Vector3d> pts_gt;

extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int FLOW_BACK;

// LiDAR
extern int NUM_OF_LASER;
extern int N_SCANS;

extern int IDX_REF;

extern int WINDOW_SIZE;
extern int OPT_WINDOW_SIZE;

extern float SCAN_PERIOD;
extern float DISTANCE_SQ_THRESHOLD;
extern float NEARBY_SCAN;

extern std::string CLOUD0_TOPIC, CLOUD1_TOPIC;
extern float LASER_SYNC_THRESHOLD;
extern double ROI_RANGE;

extern std::vector<Eigen::Matrix3d> RBL;
extern std::vector<Eigen::Quaterniond> QBL;
extern std::vector<Eigen::Vector3d> TBL;
extern std::vector<double> TDBL;

extern int PLANAR_MOVEMENT;

extern float MIN_MATCH_SQ_DIS;
extern float MIN_PLANE_DIS;

extern int MARGINALIZATION_FACTOR;
extern int POINT_DISTANCE_FACTOR;

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
    double score_;
    Eigen::Vector3d point_;
    Eigen::Vector4d coeffs_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
