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

#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "common/algos/math.hpp"

using namespace common;

// a regularized term
class PriorFactor : public ceres::SizedCostFunction<6, 7>
{
public:
    PriorFactor() = delete;
    PriorFactor(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot, const double &pos_scale, const double &rot_scale)
        : pos_(pos), rot_(rot), pos_scale_(pos_scale), rot_scale_(rot_scale) {}

    // TODO: jacobians derivation
    // residual = [t1 - t2, 2 * q1.conjugate * q2]
    bool Evaluate(double const *const *param, double *residuals, double **jacobians) const
    {
        Eigen::Quaterniond Q(param[0][6], param[0][3], param[0][4], param[0][5]);
        Eigen::Vector3d P(param[0][0], param[0][1], param[0][2]);

        Eigen::Matrix<double, 6, 6> sqrt_info; // information matrix
        sqrt_info.setZero();
        sqrt_info.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * pos_scale_;
        sqrt_info.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * rot_scale_;

        Eigen::Map<Eigen::Matrix<double, 6, 1> > residual(residuals);
        residual.topRows<3>() = P - pos_;
        residual.bottomRows<3>() = 2 * (rot_.inverse() * Q).coeffs().head<3>(); // 6*1 [dx dy dz dqx dqy dqz]

        // FIXME: info
        residual = sqrt_info * residual;
        // std::cout << "prior factor residual: " << residual.transpose() << std::endl;

        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > jacobian_prior(jacobians[0]);
                Eigen::Matrix<double, 6, 6> jaco_prior;
                jaco_prior.setIdentity();

                jaco_prior.bottomRightCorner<3, 3>() = LeftQuatMatrix(Q.inverse() * rot_).topLeftCorner<3, 3>();

                // FIXME: info
                jacobian_prior.setZero();
                jacobian_prior.leftCols<6>() = sqrt_info * jaco_prior;
                jacobian_prior.rightCols<1>().setZero();
            }
        }
        return true;
    }

    void Check(double const *const *param)
    {
        Eigen::Quaterniond Q(param[0][6], param[0][3], param[0][4], param[0][5]);
        Eigen::Vector3d P(param[0][0], param[0][1], param[0][2]);

        double *res = new double[6];
        double **jaco = new double *[1];
        jaco[0] = new double[6 * 7];
        Evaluate(param, res, jaco);
        std::cout << "check begins" << std::endl;
        std::cout << "analytical" << std::endl;

        std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(res).transpose() << std::endl;
        std::cout << std::endl << Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>>(jaco[0]) << std::endl;

        double info = 1;
        Eigen::Matrix<double, 6, 1> residual;
        residual.topRows<3>() = P - pos_;
        residual.bottomRows<3>() = 2 * (rot_.inverse() * Q).coeffs().head<3>();

        residual = info * residual;
        std::cout << "num" << std::endl;
        std::cout << residual.transpose() << std::endl;

        const double eps = 1e-6;
        Eigen::Matrix<double, 6, 7> num_jacobian;
        for (int k = 0; k < 6; k++)
        {
            Eigen::Quaterniond Q(param[0][6], param[0][3], param[0][4], param[0][5]);
            Eigen::Vector3d P(param[0][0], param[0][1], param[0][2]);

            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                P += delta;
            else if (a == 1)
                Q = Q * DeltaQ(delta);

            Eigen::Matrix<double, 6, 1> tmp_residual;

            tmp_residual.topRows<3>() = P - pos_;
            tmp_residual.bottomRows<3>() = 2 * (rot_.inverse() * Q).coeffs().head<3>();

            // FIXME: info
            tmp_residual = info * tmp_residual;
            num_jacobian.col(k) = (tmp_residual - residual) / eps;
        }
        std::cout << std::endl << num_jacobian.block<6, 6>(0, 0);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const Eigen::Vector3d pos_;
    const Eigen::Quaterniond rot_;
    const double pos_scale_;
    const double rot_scale_;
};

//
