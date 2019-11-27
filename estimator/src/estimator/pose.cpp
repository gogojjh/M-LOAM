/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose.h"

void computeMeanPose(const std::vector<std::pair<double, Pose> > &pose_array, Pose &pose_mean)
{
    // Solution 1: approximation if the separate quaternions are relatively close to each other.
    // TODO: Solution 2: using pose graph optimization T_mean = argmin_{T} \sum||(T-T_mean)||^{2}
    double weight_total = 0;
    for (auto iter = pose_array.begin(); iter != pose_array.end(); iter++)  weight_total += iter->first;

    // translation averaging
    Eigen::Vector3d t_mean = Eigen::Vector3d::Zero();
    for (auto iter = pose_array.begin(); iter != pose_array.end(); iter++)
    {
        std::cout << iter->first << ", " << iter->second << std::endl;
        t_mean += iter->first * iter->second.t_;
    }
    t_mean /= weight_total;

    // rotation averaging
    double x = 0, y = 0, z = 0, w = 0;
    for (auto iter = pose_array.begin(); iter != pose_array.end(); iter++)
    {
        x += iter->first * iter->second.q_.x();
        y += iter->first * iter->second.q_.y();
        z += iter->first * iter->second.q_.z();
        w += iter->first * iter->second.q_.w();
    }
    x /= weight_total;
    y /= weight_total;
    z /= weight_total;
    w /= weight_total;
    Eigen::Quaterniond q_mean(w, x, y, z);

    pose_mean = Pose(q_mean, t_mean);
}

Pose Pose::poseTransform(const Pose &pose1, const Pose &pose2)
{
    // t12 = t1 + q1 * t2;
    // q12 = q1 * q2;
    return Pose(pose1.q_*pose2.q_, pose1.q_*pose2.t_+pose1.t_);
}

Pose Pose::inverse()
{
    return Pose(q_.inverse(), -(q_.inverse()*t_));
}

Pose Pose::operator * (const Pose &pose)
{
    return Pose(q_*pose.q_, q_*pose.t_+t_);
}

ostream & operator << (ostream &out, const Pose &pose)
{
    out << "t: [" << pose.t_.transpose() << "], q: ["
        << pose.q_.x() << " " << pose.q_.y() << " " << pose.q_.z() << " " << pose.q_.w() << "], td: "
        << pose.td_;
    return out;
}
