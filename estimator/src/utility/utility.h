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

#pragma once

#include <cmath>
#include <cassert>
#include <cstring>

#include <ceres/ceres.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#include "common/common.hpp"
#include "common/types/type.h"
#include "common/random_generator.hpp"

#include "../estimator/parameters.h"
#include "../estimator/pose.h"

template <typename PointType>
void roiCloudFilter(pcl::PointCloud<PointType> &laser_cloud, const double &roi_range)
{
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(laser_cloud, laser_cloud, indices);
    if (roi_range <= 1e-5)
    {
        return;
    }
    else if (roi_range < 5)
    {
        common::removeROIPointCloud(laser_cloud, laser_cloud, roi_range, "inside");
    }
    else
    {
        common::removeROIPointCloud(laser_cloud, laser_cloud, roi_range, "outside");
    }
}

// project all distorted points on the last frame
// a: last frame; c: frame for points capturing
// p^a = T(s)*p^c
template <typename PointType>
inline void TransformToStart(const PointType &pi, PointType &po, const Pose &pose,
                             const bool &b_distortion, const float &scan_period = 0.1)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        std::cerr << "[TransformToStart] Point does not have intensity field!" << std::endl;
        exit(EXIT_FAILURE);
    }
    double s = 1.0; //interpolation ratio
    if (b_distortion)
        s = (pi.intensity - int(pi.intensity)) / scan_period;
    po = pi;
    // spherically interpolates between q1 and q2 by the interpolation coefficient t
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, pose.q_);
    Eigen::Vector3d t_point_last = s * pose.t_;
    Eigen::Vector3d point(pi.x, pi.y, pi.z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;
    
    po.x = un_point.x();
    po.y = un_point.y();
    po.z = un_point.z();
    po.intensity = pi.intensity;
}

// project all distorted lidar points on the current frame
// a: last frame; b: current frame; c: frame for points capturing
// p^a = T(s)*p^c, p^b = T^(-1)*T(s)*p^c
template <typename PointType>
inline void TransformToEnd(const PointType &pi, PointType &po, const Pose &pose,
                           const bool &b_distortion, const float &scan_period = 0.1)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        std::cerr << "[TransformToEnd] Point does not have intensity field!" << std::endl;
        exit(EXIT_FAILURE);
    }
    PointType un_point_tmp;
    TransformToStart(pi, un_point_tmp, pose, b_distortion, scan_period);
    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = pose.q_.inverse() * (un_point - pose.t_);
    
    po.x = point_end.x();
    po.y = point_end.y();
    po.z = point_end.z();
    po.intensity = pi.intensity;
}

template <typename PointType>
inline void pointAssociateToMap(const PointType &pi, PointType &po, const Pose &pose)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        std::cerr << "[pointAssociateToMap] Point does not have intensity field!" << std::endl;
        exit(EXIT_FAILURE);
    }
    po = pi;
    Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
    Eigen::Vector3d point_trans = pose.q_ * point_curr + pose.t_;
    po.x = point_trans.x();
    po.y = point_trans.y();
    po.z = point_trans.z();
    po.intensity = pi.intensity;
}

template <typename PointType>
inline void pointAssociateTobeMapped(const PointType &pi, PointType &po, const Pose &pose)
{
    if (!pcl::traits::has_field<PointType, pcl::fields::intensity>::value)
    {
        std::cerr << "[pointAssociateTobeMapped] Point does not have intensity field!" << std::endl;
        exit(EXIT_FAILURE);
    }
    po = pi;
    Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
    Eigen::Vector3d point_trans = pose.q_.inverse() * (point_curr - pose.t_);
    po.x = point_trans.x();
    po.y = point_trans.y();
    po.z = point_trans.z();
    po.intensity = pi.intensity;
}

template <typename T>
inline void CRSMatrix2EigenMatrix(const ceres::CRSMatrix &crs_matrix, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &eigen_matrix)
{
    eigen_matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(crs_matrix.num_rows, crs_matrix.num_cols);
    for (auto row = 0; row < crs_matrix.num_rows; row++)
    {
        int start = crs_matrix.rows[row];
        int end = crs_matrix.rows[row + 1] - 1;
        for (auto i = start; i <= end; i++)
        {
            int col = crs_matrix.cols[i];
            eigen_matrix(row, col) = T(crs_matrix.values[i]);
        }
    }
}

template <typename T>
inline void CRSMatrix2EigenMatrix(const ceres::CRSMatrix &crs_matrix, Eigen::SparseMatrix<T, Eigen::RowMajor> &eigen_matrix)
{
    eigen_matrix.resize(crs_matrix.num_rows, crs_matrix.num_cols);
    for (auto row = 0; row < crs_matrix.num_rows; row++)
    {
        int start = crs_matrix.rows[row];
        int end = crs_matrix.rows[row + 1] - 1;
        for (auto i = start; i <= end; i++)
        {
            int col = crs_matrix.cols[i];
            eigen_matrix.coeffRef(row, col) = T(crs_matrix.values[i]);
        }
    }
}

class Utility
{
  public:
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }

    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
    {
        Eigen::Matrix3d R0;
        Eigen::Vector3d ng1 = g.normalized();
        Eigen::Vector3d ng2{0, 0, 1.0};
        R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
        double yaw = Utility::R2ypr(R0).x();
        R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
        // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
        return R0;
    }

    template <size_t N>
    struct uint_
    {
    };

    template <size_t N, typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<N>)
    {
        unroller(f, iter, uint_<N - 1>());
        f(iter + N);
    }

    template <typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<0>)
    {
        f(iter);
    }

    template <typename Scalar>
    static Scalar normalizeAngle(const Scalar& angle_degrees)
    {
        Scalar two_pi(2.0 * 180);
        if (angle_degrees > 0)
            return angle_degrees - two_pi * std::floor((angle_degrees + Scalar(180)) / two_pi);
        else
            return angle_degrees + two_pi * std::floor((-angle_degrees + Scalar(180)) / two_pi);
    }
};

//
