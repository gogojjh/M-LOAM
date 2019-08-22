/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *
 * Reference: Monocular visual-inertial fusion with online initialization and camera-IMU calibration
 *******************************************************/

#include "initial_extrinsics.h"

using namespace Eigen;

double EPSILON_R = 0.01;
double EPSILON_T = 0.01;

InitialExtrinsics::InitialExtrinsics()
{
    calib_bl_.resize(NUM_OF_LASER);

    cov_rot_state_ = std::vector<bool>(NUM_OF_LASER, false);
    full_cov_rot_state_ = false;
}

void InitialExtrinsics::clearState()
{
    calib_bl_.resize(NUM_OF_LASER);

    cov_rot_state_ = std::vector<bool>(NUM_OF_LASER, false);
    full_cov_rot_state_ = false;
}

bool InitialExtrinsics::setCovRotation(const size_t &idx)
{
    assert(idx < cov_rot_state_.size());
    cov_rot_state_[idx] = true;
    if (std::find(cov_rot_state_.begin(), cov_rot_state_.end(), false) == cov_rot_state_.end())
    {
        full_cov_rot_state_ = true;
    }
}

bool InitialExtrinsics::checkScrewMotion(const Pose &pose_ref, const Pose &pose_data)
{
    AngleAxisd ang_axis_ref(pose_ref.q_);
    AngleAxisd ang_axis_data(pose_data.q_);
    double r_dis = abs(ang_axis_ref.angle() - ang_axis_data.angle());
    double t_dis = abs(pose_ref.t_.dot(ang_axis_ref.axis()) - pose_data.t_.dot(ang_axis_data.axis()));
    v_rd_.push_back(r_dis);
    v_td_.push_back(t_dis);
    if ((r_dis < EPSILON_R) && (t_dis < EPSILON_T))
        return true;
    else
        return false;
}

bool InitialExtrinsics::calibExRotation(
    const std::vector<Pose> &v_pose_ref,
    const std::vector<Pose> &v_pose_data,
    const size_t &idx,
    Pose &calib_result)
{
    // -------------------------------
    // screw motion filter
    assert(v_pose_ref.size() == v_pose_data.size());
    assert(idx < cov_rot_state_.size());

    std::vector<Pose> v_pose_ref_filter, v_pose_data_filter;
    v_pose_ref_filter.resize(v_pose_ref.size());
    v_pose_data_filter.resize(v_pose_ref.size());
    v_rd_.resize(v_pose_ref.size());
    v_td_.resize(v_pose_ref.size());

    size_t j = 0;
    for (size_t i = 0; i < v_pose_ref.size(); i++)
    {
        double ad;
        if (checkScrewMotion(v_pose_ref[i], v_pose_data[i]))
        {
            v_pose_ref_filter[j] = v_pose_ref[i];
            v_pose_data_filter[j] = v_pose_data[i];
            j++;
        }
    }
    if (j != v_pose_ref.size())
    {
        v_pose_ref_filter.resize(j);
        v_pose_data_filter.resize(j);
    }
    size_t frame_cnt = v_pose_ref_filter.size();

    // -------------------------------
    // initial rotation
    MatrixXd A(frame_cnt * 4, 4); // a cumulative Q matrix
    A.setZero();
    for (int i = 0; i < frame_cnt; i++)
    {
        Pose &pose_ref = v_pose_ref_filter[i];
        Pose &pose_data = v_pose_data_filter[i];

        Quaterniond r1 = pose_ref.q_;
        Quaterniond r2 = calib_bl_[idx].q_ * pose_data.q_ * calib_bl_[idx].inverse().q_;
        double angular_distance = 180 / M_PI * r1.angularDistance(r2);
        ROS_DEBUG("%d %f", i, angular_distance);
        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;

        Matrix4d L, R; // Q1 and Q2 to represent the quaternion representation
        double w = pose_ref.q_.w();
        Vector3d q = pose_ref.q_.vec();
        L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        w = pose_data.q_.w();
        q = pose_data.q_.vec();
        R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }

    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    Matrix<double, 4, 1> x = svd.matrixV().col(3);
    // Quaterniond estimated_R(x);
    // rbl[idx] = estimated_R.toRotationMatrix().inverse();
    //cout << svd.singularValues().transpose() << endl;
    //cout << ric << endl;
    calib_bl_[idx].q_ = Quaterniond(x);
    Vector3d rot_cov = svd.singularValues().tail<3>(); // singular value
    if (frame_cnt >= WINDOW_SIZE && rot_cov(1) > 0.25)
    {
        calib_result = calib_bl_[idx];
        return true;
    }
    else
        return false;
}

void InitialExtrinsics::decomposeE(cv::Mat E,
                                 cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                 cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}

void InitialExtrinsics::saveScrewMotion(const std::string &filename)
{
    try
    {
        csvfile csv(filename.c_str());
        // Hearer
        csv << "i" << "rd" << "td" << endrow;
        // Data
        for (size_t i = 0; i < v_rd_.size(); i++)
        {
            csv << i << v_rd_[i] << v_td_[i] << endrow;
        }
    }
    catch (const std::exception &ex)
    {
        std::cout << "Exception was thrown when save screw motion: " << ex.what() << std::endl;
    }
    return;
}
