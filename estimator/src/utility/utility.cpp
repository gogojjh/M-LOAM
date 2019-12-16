/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/
#include "utility.h"

using namespace common;

// undistort lidar point
void TransformToStart(PointI const *const pi, PointI *const po, const Pose &pose, const bool &b_distortion)
{
    //interpolation ratio
    double s;
    if (b_distortion)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, pose.q_);
    Eigen::Vector3d t_point_last = s * pose.t_;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame
void TransformToEnd(PointI const *const pi, PointI *const po, const Pose &pose, const bool &b_distortion)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp, pose, b_distortion);
    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = pose.q_.inverse() * (un_point - pose.t_);
    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();
    po->intensity = int(pi->intensity);
}

void pointAssociateToMap(const PointI &pi, PointI &po, const Pose &pose)
{
    Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
    Eigen::Vector3d point_trans = pose.q_ * point_curr + pose.t_;
    po.x = point_trans.x();
    po.y = point_trans.y();
    po.z = point_trans.z();
    po.intensity = pi.intensity;
}

void pointAssociateTobeMapped(const PointI &pi, PointI &po, const Pose &pose)
{
    Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
    Eigen::Vector3d point_trans = pose.q_.inverse() * (point_curr - pose.t_);
    po.x = point_trans.x();
    po.y = point_trans.y();
    po.z = point_trans.z();
    po.intensity = pi.intensity;
}

//
