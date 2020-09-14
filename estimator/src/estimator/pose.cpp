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

#include "pose.h"

Pose::Pose()
{
    q_ = Eigen::Quaterniond::Identity();
    t_ = Eigen::Vector3d::Zero();
    T_ = Eigen::Matrix4d::Identity();
    td_ = 0;
    cov_.setZero();
}

Pose::Pose(const Pose &pose)
{
    q_ = pose.q_;
    t_ = pose.t_;
    T_ = pose.T_;
    td_ = pose.td_;
    cov_ = pose.cov_;
}

Pose::Pose(const Eigen::Quaterniond &q, const Eigen::Vector3d &t, const double &td)
{
    q_ = q; q_.normalize();
    t_ = t;
    T_.setIdentity(); T_.topLeftCorner<3, 3>() = q_.toRotationMatrix(); T_.topRightCorner<3, 1>() = t_;
    td_ = td;
    cov_.setZero();
}

Pose::Pose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, const double &td)
{
    q_ = Eigen::Quaterniond(R);
    t_ = t;
    T_.setIdentity(); T_.topLeftCorner<3, 3>() = R; T_.topRightCorner<3, 1>() = t_;
    td_ = td;
    cov_.setZero();
}

Pose::Pose(const Eigen::Matrix4d &T, const double &td)
{
    q_ = Eigen::Quaterniond(T.topLeftCorner<3, 3>()); q_.normalize();
    t_ = T.topRightCorner<3, 1>();
    T_ = T;
    td_ = td;
    cov_.setZero();
}

Pose::Pose(const nav_msgs::Odometry &odom)
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
    T_.setIdentity(); T_.topLeftCorner<3, 3>() = q_.toRotationMatrix(); T_.topRightCorner<3, 1>() = t_;
    for (size_t i = 0; i < 6; i++)
        for (size_t j = 0; j < 6; j++)
            cov_(i, j) = double(odom.pose.covariance[i * 6 + j]);
}

Pose::Pose(const geometry_msgs::Pose &pose)
{
    q_ = Eigen::Quaterniond(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    t_ = Eigen::Vector3d(
        pose.position.x,
        pose.position.y,
        pose.position.z);
    T_.setIdentity(); T_.topLeftCorner<3, 3>() = q_.toRotationMatrix(); T_.topRightCorner<3, 1>() = t_;
}

Pose Pose::poseTransform(const Pose &pose1, const Pose &pose2)
{
    // t12 = t1 + q1 * t2;
    // q12 = q1 * q2;
    return Pose(pose1.q_*pose2.q_, pose1.q_*pose2.t_+pose1.t_);
}

Pose Pose::inverse() const
{
    return Pose(q_.inverse(), -(q_.inverse()*t_));
}

void Pose::update() 
{
    T_.topLeftCorner<3, 3>() = q_.toRotationMatrix(); 
    T_.topRightCorner<3, 1>() = t_;
}

Pose Pose::operator * (const Pose &pose)
{
    return Pose(q_*pose.q_, q_*pose.t_+t_);
}

ostream & operator << (ostream &out, const Pose &pose)
{
    out << std::fixed << std::setprecision(3)
        << "t [" << pose.t_(0) << "," << pose.t_(1) << "," << pose.t_(2) 
        << "], q [" << pose.q_.x() << "," << pose.q_.y() << "," << pose.q_.z() << "," << pose.q_.w()
        << "], td [" << pose.td_ << "]";
    return out;
}

Eigen::Matrix<double, 6, 1> Pose::se3() const 
{
    Sophus::SE3d SE3_qt(q_, t_);
    Eigen::Matrix<double, 6, 1> xi = SE3_qt.log();
    // Sophus::SE3::hat(xi)
    // Sophus::SE3::vee(Sophus::SE3::hat(xi)) == xi
    return xi;
}

// quaternion averaging: https://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors
// ceres: pose graph optimization: https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_3d
// review of rotation averaging: Rotation Averaging IJCV
// TODO: Solution 2: using pose graph optimization T_mean = argmin_{T} \sum||(T-T_mean)||^{2}
void computeMeanPose(const std::vector<std::pair<double, Pose> > &pose_array, 
                     Pose &pose_mean, Eigen::Matrix<double, 6, 6> &pose_cov)
{
    // Solution 1: approximation if the separate quaternions are relatively close to each other.
    // Solution 2: compute the mean in se3
    pose_cov.setZero();
    if (pose_array.size() == 1)
    {
        pose_mean = pose_array[0].second;
        return;
    }

    const std::vector<std::pair<double, Pose>> pose_array_filter(
        pose_array.begin() + size_t(pose_array.size() / 3), pose_array.end());

    // double weight_total = 0;
    // Eigen::Matrix<double, 6, 1> xi_total = Eigen::Matrix<double, 6, 1>::Zero();
    // Eigen::Matrix<double, 6, 1> xi_mean = xi_total / weight_total;
    Eigen::Matrix<double, 6, 1> xi_mean = Eigen::Matrix<double, 6, 1>::Zero();
    for (auto iter = pose_array_filter.begin(); iter != pose_array_filter.end(); iter ++)
    {
        // weight_total += iter->first;
        // xi_total += iter->first * iter->second.se3();
        // std::cout << iter->first << ", " << iter->second << std::endl;
        // LOG(INFO) << iter->first << ", " << iter->second;
        std::cout << iter->second << std::endl;
        LOG(INFO) << iter->second;
        xi_mean += iter->second.se3() / pose_array_filter.size();
    }
    pose_mean = Pose(Sophus::SE3d::exp(xi_mean).matrix());

    for (auto iter = pose_array_filter.begin(); iter != pose_array_filter.end(); iter++)
    {
        Eigen::Matrix<double, 6, 1> xi = iter->second.se3();
        // pose_cov += iter->first * iter->first * (xi - xi_mean) * (xi - xi_mean).transpose();
        pose_cov += (pose_array_filter.size() - 1) * (xi - xi_mean) * (xi - xi_mean).transpose();
    }

    std::cout << "calib mean: " << pose_mean << std::endl;
    std::cout << "calib cov: " << std::endl << pose_cov << std::endl;
    LOG(INFO) << "calib mean: " << pose_mean;
    LOG(INFO) << "calib cov: " << std::endl << pose_cov;
}

// 