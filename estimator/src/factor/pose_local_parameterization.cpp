/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose_local_parameterization.h"

void PoseLocalParameterization::setParameter()
{
    is_degenerate_ = false;
    V_p_ = Eigen::Matrix<double, 6, 6>::Identity();
}

// state update
// about the update rule: LIC-Fusion: LiDAR-Inertial-Camera Odometry, IROS 2019
// delta = [dp, dq]
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);
    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3)); // using theta to approximate q

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    // if (is_degenerate_)
    //     V_p_ * dp   V_p_ * dq

    p = _p + dp;
    q = (_q * dq).normalized(); // q = _q * [0.5*delta, 1]

    return true;
}

// calculate the jacobian of [p, q] w.r.t [dp, dq]
// turtial: https://blog.csdn.net/hzwwpgmwy/article/details/86490556
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();
    return true;
}




//
