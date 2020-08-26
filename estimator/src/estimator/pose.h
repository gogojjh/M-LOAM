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

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <map>
#include <iomanip>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

#include "common/types/type.h"

using namespace std;

class Pose
{
public:
    Pose();
    Pose(const Pose &pose);
    Pose(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, const double &td=0);
    Pose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, const double &td=0);
    Pose(const Eigen::Matrix4d &T, const double &td=0);
    Pose(const nav_msgs::Odometry &odom);
    Pose(const geometry_msgs::Pose &pose);

    static Pose poseTransform(const Pose &pose1, const Pose &pose2);

    void update();

    Pose inverse() const;
    Pose operator * (const Pose &pose);
    friend ostream &operator << (ostream &out, const Pose &pose);

    Eigen::Matrix<double, 6, 1> se3() const;

    double td_;
    Eigen::Quaterniond q_; // q = [cos(theta/2), u*sin(theta/2)]
    Eigen::Vector3d t_;
    Eigen::Matrix4d T_;
    Eigen::Matrix<double, 6, 6> cov_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // TODO: the Eigen bugs in initializing the class
};

void computeMeanPose(const std::vector<std::pair<double, Pose> > &pose_array, Pose &pose_mean, Eigen::Matrix<double, 6, 6> &pose_cov);

//
