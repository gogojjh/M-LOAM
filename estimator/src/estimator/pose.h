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

#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>

#include "common/types/type.h"

using namespace std;

class Pose
{
public:
    Pose(): q_(Eigen::Quaterniond::Identity()), t_(Eigen::Vector3d::Zero()), T_(Eigen::Matrix4d::Identity()), td_(0) {}
    Pose(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, const double &td=0): q_(q), t_(t), td_(td)
    {
        q_.normalize();
        T_.setIdentity();
        T_.topLeftCorner<3, 3>() = q_.toRotationMatrix();
        T_.topRightCorner<3, 1>() = t_;
    }
    Pose(const Eigen::Matrix4d &T, const double &td=0): T_(T), td_(td)
    {
        q_ = Eigen::Quaterniond(T.topLeftCorner<3, 3>());
        q_.normalize();
        t_ = T.topRightCorner<3, 1>();
    }
    Pose(const nav_msgs::Odometry &odom)
    {
        q_ = Eigen::Quaterniond(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z);
        t_ = Eigen::Vector3d(
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z);
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
    // q = [cos(theta/2), u*sin(theta/2)]
    Eigen::Vector3d t_;
    Eigen::Matrix4d T_;

    double td_;
};
