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

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"

// calculate distrance from point to plane (using normal)
struct LidarScanPlaneNormFactor 
{
public:
    LidarScanPlaneNormFactor(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, const double &s)
        : point_(point), coeff_(coeff), s_(s) {}

    template <typename T>
    bool operator()(const T *param, T *residual) const 
    {
        Eigen::Matrix<T, 3, 1> cp{T(point_[0]), T(point_[1]), T(point_[2])};

        Eigen::Quaternion<T> q_last_curr{param[6], param[3], param[4], param[5]};
        Eigen::Matrix<T, 3, 1> t_last_curr{param[0], param[1], param[2]};

        Eigen::Matrix<T, 3, 1> w{T(coeff_(0)), T(coeff_(1)), T(coeff_(2))};
        T d = T(coeff_(3));
        residual[0] = w.dot(q_last_curr * cp + t_last_curr) + d;

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d point, 
                                       const Eigen::Vector4d coeff,
                                       const double s)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarScanPlaneNormFactor, 1, 7>(
            new LidarScanPlaneNormFactor(point, coeff, s)));
    }

private:
    Eigen::Vector3d point_;
    Eigen::Vector4d coeff_;
    double s_;
};
