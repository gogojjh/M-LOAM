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

InitialExtrinsics::InitialExtrinsics() {}

void InitialExtrinsics::clearState()
{
    calib_bl_.clear();
    cov_rot_state_.clear();
    v_rot_cov_.clear();
}

void InitialExtrinsics::setParameter()
{
    calib_bl_.resize(NUM_OF_LASER);
    for (size_t i = 0; i < NUM_OF_LASER; i++) calib_bl_[i] = Pose(QBL[i], TBL[i], TDBL[i]);

    cov_rot_state_.resize(NUM_OF_LASER);
    for (size_t i = 0; i < cov_rot_state_.size(); i++) cov_rot_state_[i] = false;
    full_cov_rot_state_ = false;

    v_rot_cov_.resize(NUM_OF_LASER);

    frame_cnt_ = 0;
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
    // std::cout << "ref pose : " << pose_ref << std::endl;
    // std::cout << "data pose : " << pose_data << std::endl;
    // printf("r_dis: %f, t_dis: %f \n", r_dis, t_dis);
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

    printf("frame_cnt_ before: %d\n", v_pose_ref.size());

    std::vector<Pose> v_pose_ref_filter, v_pose_data_filter;
    v_pose_ref_filter.resize(v_pose_ref.size());
    v_pose_data_filter.resize(v_pose_ref.size());
    v_rd_.resize(v_pose_ref.size());
    v_td_.resize(v_pose_ref.size());

    size_t j = 0;
    for (size_t i = 0; i < v_pose_ref.size(); i++)
    {
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

    frame_cnt_ = v_pose_ref_filter.size();
    printf("frame_cnt_ after: %d\n", frame_cnt_);

    // -------------------------------
    // initial rotation
    TicToc t_calib_rot;
    Eigen::MatrixXd A(frame_cnt_ * 4, 4); // a cumulative Q matrix
    A.setZero();
    for (int i = 0; i < frame_cnt_; i++)
    {
        Pose &pose_ref = v_pose_ref_filter[i];
        Pose &pose_data = v_pose_data_filter[i];

        Eigen::Quaterniond r1 = pose_ref.q_;
        Eigen::Quaterniond r2 = calib_bl_[idx].q_ * pose_data.q_ * calib_bl_[idx].inverse().q_;
        double angular_distance = 180 / M_PI * r1.angularDistance(r2);
        ROS_DEBUG("%d %f", i, angular_distance);
        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;

        Eigen::Matrix4d L, R; // Q1 and Q2 to represent the quaternion representation
        double w = pose_ref.q_.w();
        Eigen::Vector3d q = pose_ref.q_.vec();
        L.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        w = pose_data.q_.w();
        q = pose_data.q_.vec();
        R.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        A.block<4, 4>(i * 4, 0) = huber * (L - R);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //cout << svd.singularValues().transpose() << endl;

    if (PLANAR_MOVEMENT)
    {
        // estimateRyx
        Eigen::Vector4d t1 = svd.matrixV().block<4,1>(0,2);
        Eigen::Vector4d t2 = svd.matrixV().block<4,1>(0,3);
        // solve constraint for q_yz: xy = -zw
        double s[2];
        if (!common::solveQuadraticEquation(t1(0) * t1(1) + t1(2) * t1(3),
                                        t1(0) * t2(1) + t1(1) * t2(0) + t1(2) * t2(3) + t1(3) * t2(2),
                                        t2(0) * t2(1) + t2(2) * t2(3),
                                        s[0], s[1]))
        {
            std::cout << "# ERROR: Quadratic equation cannot be solved due to negative determinant." << std::endl;
            return false;
        }
        std::vector<Eigen::Quaterniond> q_yx;
        q_yx.resize(2);
        double yaw[2];
        for (int i = 0; i < 2; ++i)
        {
            double t = s[i] * s[i] * t1.dot(t1) + 2 * s[i] * t1.dot(t2) + t2.dot(t2);
            // solve constraint ||q_yx|| = 1
            double b = sqrt(1.0 / t);
            double a = s[i] * b;
            q_yx[i].coeffs() = a * t1 + b * t2; // [w x y z]
            Eigen::Vector3d euler_angles = q_yx[i].toRotationMatrix().eulerAngles(2, 1, 0);
            yaw[0] = euler_angles(0);
        }
        if (fabs(yaw[0]) < fabs(yaw[1]))
            calib_bl_[idx].q_ = q_yx[0];
        else
            calib_bl_[idx].q_ = q_yx[1];
    } else
    {
        Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
        calib_bl_[idx].q_ = Eigen::Quaterniond(x);
    }

    Vector3d rot_cov = svd.singularValues().tail<3>(); // singular value
    printf("calib ext rot: %f ms\n", t_calib_rot.toc());
    v_rot_cov_[idx].push_back(rot_cov(1));

    if (frame_cnt_ >= WINDOW_SIZE && rot_cov(1) > 0.25)
    {
        if (PLANAR_MOVEMENT)
            calibExTranslationPlanar(v_pose_ref_filter, v_pose_data_filter, idx);
        else
            calibExTranslation(v_pose_ref_filter, v_pose_data_filter, idx);

        if (ESTIMATE_TD)
            calibTimeDelay(v_pose_ref_filter, v_pose_data_filter, idx);

        calib_result = calib_bl_[idx];
        return true;
    }
    else
    {
        return false;
    }
}

void InitialExtrinsics::calibExTranslation(
    const std::vector<Pose> &v_pose_ref,
    const std::vector<Pose> &v_pose_data,
    const size_t &idx)
{
    const Eigen::Quaterniond &q_zyx = calib_bl_[idx].q_;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(frame_cnt_ * 3, 3);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(frame_cnt_ * 3, 1);
    for (size_t i = 0; i < frame_cnt_; i++)
    {
        A.block<3, 3>(i * 3, 0) = v_pose_ref[i].q_.toRotationMatrix() - Eigen::Matrix3d::Identity();
        b.block<3, 1>(i * 3, 0) = q_zyx * v_pose_data[i].t_ - v_pose_ref[i].t_;
    }
    Eigen::Vector3d x;
    x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    calib_bl_[idx] = Pose(q_zyx, x);
}

void InitialExtrinsics::calibExTranslationPlanar(
    const std::vector<Pose> &v_pose_ref,
    const std::vector<Pose> &v_pose_data,
    const size_t &idx)
{
    const Eigen::Quaterniond &q_yx = calib_bl_[idx].q_;
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(frame_cnt_ * 2, 4);
    Eigen::MatrixXd w = Eigen::MatrixXd::Zero(frame_cnt_ * 2, 1);
    for (int i = 0; i < frame_cnt_; ++i)
    {
        Eigen::Matrix2d J = v_pose_ref[i].q_.toRotationMatrix().block<2,2>(0, 0) - Eigen::Matrix2d::Identity();
        Eigen::Vector3d n = q_yx.toRotationMatrix().row(2);
        Eigen::Vector3d p = q_yx * (v_pose_data[i].t_ - v_pose_data[i].t_.dot(n) * n);
        Eigen::Matrix2d K;
        K << p(0), -p(1), p(1), p(0);
        G.block<2,2>(i * 2, 0) = J;
        G.block<2,2>(i * 2, 2) = K;
        w.block<2,1>(i * 2, 0) = v_pose_ref[i].t_.block<2,1>(0,0);
    }
    Eigen::Vector4d m = G.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(w);
    Eigen::Vector3d x(-m(0), -m(1), 0);
    double yaw = atan2(m(3), m(2));
    Eigen::Quaterniond q_z(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    calib_bl_[idx] = Pose(q_z*q_yx, x);
}

void InitialExtrinsics::calibTimeDelay(
    const std::vector<Pose> &v_pose_ref,
    const std::vector<Pose> &v_pose_data,
    const size_t &idx)
{

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

// save all poses of all lasers in initialization
void InitialExtrinsics::saveStatistics(const std::vector<std::vector<Pose> > &v_pose)
{
    try
    {
        ofstream fout(std::string(OUTPUT_FOLDER + "initialization.csv").c_str(), ios::out);
        fout.setf(ios::fixed, ios::floatfield);
        fout.precision(5);
        // orientation
        for (size_t idx = 0; idx < NUM_OF_LASER; idx ++)
        {
            for (size_t j = 0; j < v_pose[idx].size(); j++)
            {
                fout << Eigen::AngleAxisd(v_pose[idx][j].q_).angle() << ",";
                if (j == v_pose[idx].size() - 1) fout << std::endl;
            }
        }
        // rot_cov
        for (size_t idx = 0; idx < NUM_OF_LASER; idx ++)
        {
            for (size_t j = 0; j < v_rot_cov_[idx].size(); j++)
            {
                fout << v_rot_cov_[idx][j] << ",";
                if (j == v_rot_cov_[idx].size() - 1) fout << std::endl;
            }
        }
        fout.close();
    }
    catch (const std::exception &ex)
    {
        std::cout << "Exception was thrown when save initial extrinsics: " << ex.what() << std::endl;
    }
    return;
}
