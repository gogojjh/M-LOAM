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
#include <unsupported/Eigen/MatrixFunctions>

#include "../utility/utility.h"

// calculate distrance from point to plane (using normal)
class LidarMapPlaneNormFactor : public ceres::SizedCostFunction<3, 7>
{
public:
	LidarMapPlaneNormFactor(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, const Eigen::Matrix3d &cov_matrix = Eigen::Matrix3d::Identity())
		: point_(point),
		  coeff_(coeff),
		  sqrt_info_(Eigen::LLT<Eigen::Matrix<double, 3, 3>>(cov_matrix.inverse()).matrixL().transpose())
	{
		// is a upper triangular matrix
		// std::cout << sqrt_info_ << std::endl << std::endl;
		// Eigen::matrix_sqrt_triangular(sqrt_info_, sqrt_info_);
		// std::cout << sqrt_info_ << std::endl;
		// exit(EXIT_FAILURE);
	}

	bool Evaluate(double const *const *param, double *residuals, double **jacobians) const
	{
		Eigen::Quaterniond q_w_curr(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_w_curr(param[0][0], param[0][1], param[0][2]);

		Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
		double d = coeff_(3);
		double a = w.dot(q_w_curr * point_ + t_w_curr) + d;
		Eigen::Map<Eigen::Vector3d> r(residuals);
		r = a * w;
		r.applyOnTheLeft(sqrt_info_);

		if (jacobians)
		{
			Eigen::Matrix3d R = q_w_curr.toRotationMatrix();
			Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
			for (size_t i = 0; i < W.rows(); i++) W.row(i) = w.transpose(); // [w^T;w^T;w^T]
			W = w.asDiagonal() * W;
			if (jacobians[0])
			{
				Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_pose(jacobians[0]);
				Eigen::Matrix<double, 3, 6> jaco; // [dy/dt, dy/dq, 1]
				jaco.setZero();
				jaco.leftCols<3>() = W;
				jaco.rightCols<3>() = -W * R * Utility::skewSymmetric(point_);

				jacobian_pose.setZero();
				jacobian_pose.leftCols<6>() = sqrt_info_ * jaco;
			}
		}
		return true;
	}

	void check(double **param)
	{
		double *res = new double[3];
		double **jaco = new double *[1];
		jaco[0] = new double[3 * 7];
		Evaluate(param, res, jaco);
		std::cout << "[LidarMapPlaneNormFactor] check begins" << std::endl;
        std::cout << "analytical:" << std::endl;
        std::cout << res[0] << " " << res[1] << " " << res[2] << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> >(jaco[0]) << std::endl;

		delete[] jaco[0];
		delete[] jaco;
		delete[] res;	

		Eigen::Quaterniond q_w_curr(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_w_curr(param[0][0], param[0][1], param[0][2]);

		Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
		double d = coeff_(3);
		double a = w.dot(q_w_curr * point_ + t_w_curr) + d;
		Eigen::Vector3d r = a * w;
		r.applyOnTheLeft(sqrt_info_);

        std::cout << "perturbation:" << std::endl;
        std::cout << r.transpose() << std::endl;

        const double eps = 1e-6;
        Eigen::Matrix<double, 3, 6> num_jacobian;

		// add random perturbation
		for (int k = 0; k < 6; k++)
		{
			Eigen::Quaterniond q_w_curr(param[0][6], param[0][3], param[0][4], param[0][5]);
			Eigen::Vector3d t_w_curr(param[0][0], param[0][1], param[0][2]);
			int a = k / 3, b = k % 3;
			Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;
			if (a == 0)
				t_w_curr += delta;
			else if (a == 1)
				q_w_curr = q_w_curr * Utility::deltaQ(delta);

			Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
	        double d = coeff_(3);
			double s =  w.dot(q_w_curr * point_ + t_w_curr) + d;
			Eigen::Vector3d tmp_r = s * w;
	        tmp_r.applyOnTheLeft(sqrt_info_);
            num_jacobian.col(k) = (tmp_r - r) / eps;
        }
        std::cout << num_jacobian.block<1, 6>(0, 0) << std::endl;
		std::cout << num_jacobian.block<1, 6>(1, 0) << std::endl;
		std::cout << num_jacobian.block<1, 6>(2, 0) << std::endl;
    }

private:
	const Eigen::Vector3d point_;
	const Eigen::Vector4d coeff_;
	const Eigen::Matrix3d sqrt_info_;
};
