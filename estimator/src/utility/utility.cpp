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

// // project all distorted points on the last frame
// // a: last frame; c: frame for points capturing
// // p^a = T(s)*p^c
// void TransformToStart(const PointI &pi, PointI &po, const Pose &pose, const bool &b_distortion)
// {
//     double s; //interpolation ratio
//     if (b_distortion)
//         s = (pi.intensity - int(pi.intensity)) / SCAN_PERIOD;
//     else
//         s = 1.0;
//     // spherically interpolates between q1 and q2 by the interpolation coefficient t
//     Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, pose.q_); 
//     Eigen::Vector3d t_point_last = s * pose.t_;
//     Eigen::Vector3d point(pi.x, pi.y, pi.z);
//     Eigen::Vector3d un_point = q_point_last * point + t_point_last;
//     po.x = un_point.x();
//     po.y = un_point.y();
//     po.z = un_point.z();
//     po.intensity = pi.intensity;
// }

// // project all distorted lidar points on the current frame
// // a: last frame; b: current frame; c: frame for points capturing
// // p^a = T(s)*p^c, p^b = T^(-1)*T(s)*p^c
// void TransformToEnd(const PointI &pi, PointI &po, const Pose &pose, const bool &b_distortion)
// {
//     PointI un_point_tmp;
//     TransformToStart(pi, un_point_tmp, pose, b_distortion);
//     Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
//     Eigen::Vector3d point_end = pose.q_.inverse() * (un_point - pose.t_);
//     po.x = point_end.x();
//     po.y = point_end.y();
//     po.z = point_end.z();
//     po.intensity = pi.intensity;
// }

//
