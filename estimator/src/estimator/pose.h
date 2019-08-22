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
#include <vector>
#include <fstream>
#include <map>

#include <eigen3/Eigen/Dense>

#include "common/types/type.h"

using namespace std;

class Pose
{
public:
    Pose(): q_(Eigen::Quaterniond::Identity()), t_(Eigen::Vector3d::Zero()), T_(Eigen::Matrix4d::Identity()) {}
    Pose(const Eigen::Quaterniond &q, const Eigen::Vector3d &t): q_(q), t_(t)
    {
        T_.setIdentity();
        T_.topLeftCorner<3, 3>() = q_.toRotationMatrix();
        T_.topRightCorner<3, 1>() = t_;
    }

    static Pose poseTransform(const Pose &pose1, const Pose &pose2);

    Pose inverse();

    Pose operator * (const Pose &pose);
    // Pose operator = (const Pose &pose);
    friend ostream &operator << (ostream &out, const Pose &pose);

    Eigen::Quaterniond q_;
    Eigen::Vector3d t_;
    Eigen::Matrix4d T_;
};
