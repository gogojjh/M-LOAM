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
void TransformToStart(const PointI &pi, PointI &po, const Pose &pose, const bool &b_distortion)
{
    po = pi;
    double s; //interpolation ratio
    if (b_distortion)
        s = (pi.intensity - int(pi.intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, pose.q_);
    Eigen::Vector3d t_point_last = s * pose.t_;
    Eigen::Vector3d point(pi.x, pi.y, pi.z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;
    po.x = un_point.x();
    po.y = un_point.y();
    po.z = un_point.z();
    po.intensity = pi.intensity;
}

// transform all lidar points to original frame
void TransformToEnd(const PointI &pi, PointI &po, const Pose &pose, const bool &b_distortion)
{
    po = pi;
    PointI un_point_tmp;
    TransformToStart(pi, un_point_tmp, pose, b_distortion);
    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = pose.q_.inverse() * (un_point - pose.t_);
    po.x = point_end.x();
    po.y = point_end.y();
    po.z = point_end.z();
    po.intensity = int(pi.intensity);
}

void evalPointUncertainty(const int &idx, const PointI &pi, Eigen::Matrix3d &cov_po)
{
    Eigen::Vector4d point_curr(pi.x, pi.y, pi.z, 1);
    if (idx == IDX_REF)
    {
        cov_po = XI.bottomRightCorner<3, 3>(); // keep the covariance of points
    } else
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 4, 3> D;
        D << 1, 0, 0,
             0, 1, 0,
             0, 0, 1,
             0, 0, 0;
        Eigen::Matrix<double, 4, 9> G;
        G.block<4, 6>(0, 0) = pointToFS(T * point_curr);
        G.block<4, 3>(0, 6) = T * D;
        cov_po = Eigen::Matrix4d(G * XI * G.transpose()).topLeftCorner<3, 3>(); // 4x4
    }
    // std::cout << "evalUncertainty:" << std::endl
    //           << point_curr.transpose() << std::endl
    //           << cov_po << std::endl;
    // exit(EXIT_FAILURE);
}

// pointToFS turns a 4x1 homogeneous point into a special 4x6 matrix
Eigen::Matrix<double, 4, 6> pointToFS(const Eigen::Vector4d &point)
{
    Eigen::Matrix<double, 4, 6> G = Eigen::Matrix<double, 4, 6>::Zero();
    G.block<3, 3>(0, 0) = point(3) * Eigen::Matrix<double, 3, 3>::Identity();
    G.block<3, 3>(0, 3) = -Utility::skewSymmetric(point.block<3, 1>(0, 0));
    return G;
}

//
