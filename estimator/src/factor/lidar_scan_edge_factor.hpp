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
class LidarScanEdgeFactor : public ceres::SizedCostFunction<3, 7>
{
public:
    LidarScanEdgeFactor(const Eigen::Vector3d &point,
                        const Eigen::VectorXd &coeff,
                        const double &s = 1.0)
        : point_(point), coeff_(coeff), s_(s) {}

    bool Evaluate(double const *const *param, double *residuals, double **jacobians) const
	{
        Eigen::Quaterniond q_last_curr(param[0][6], param[0][3], param[0][4], param[0][5]);
        Eigen::Vector3d t_last_curr(param[0][0], param[0][1], param[0][2]);
        q_last_curr = Eigen::Quaterniond::Identity().slerp(s_, q_last_curr);
        t_last_curr = s_ * t_last_curr;

        Eigen::Vector3d lpa(coeff_(0), coeff_(1), coeff_(2));
		Eigen::Vector3d lpb(coeff_(3), coeff_(4), coeff_(5));
		Eigen::Vector3d lp = q_last_curr * point_ + t_last_curr;

		Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
		Eigen::Vector3d de = lpa - lpb;
		residuals[0] = nu.x() / de.norm();
		residuals[1] = nu.y() / de.norm();
		residuals[2] = nu.z() / de.norm();

		if (jacobians)
		{
			Eigen::Matrix3d R = q_last_curr.toRotationMatrix();
			if (jacobians[0])
			{
				Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_pose(jacobians[0]);
				Eigen::Matrix<double, 3, 6> jaco; // [dy/dt, dy/dq, 1]

				double eta = 1.0 / de.norm();
				jaco.leftCols<3>() = -eta * Utility::skewSymmetric(lpa - lpb);
				jaco.rightCols<3>() = eta * Utility::skewSymmetric(lpa - lpb) * Utility::skewSymmetric(R * point_);

				jacobian_pose.setZero();
				jacobian_pose.leftCols<6>() = jaco;
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
		std::cout << "[LidarScanEdgeFactor] check begins" << std::endl;
        std::cout << "analytical:" << std::endl;
		std::cout << res[0] << " " << res[1] << " " << res[2] << std::endl;
		std::cout << Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> >(jaco[0]) << std::endl;

		delete[] jaco[0];
		delete[] jaco;
		delete[] res;

		Eigen::Quaterniond q_w_curr(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_w_curr(param[0][0], param[0][1], param[0][2]);

		Eigen::Vector3d lpa(coeff_(0), coeff_(1), coeff_(2));
		Eigen::Vector3d lpb(coeff_(3), coeff_(4), coeff_(5));
		Eigen::Vector3d lp = q_w_curr * point_ + t_w_curr;

		Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
		Eigen::Vector3d de = lpa - lpb;
		Eigen::Vector3d r = nu / de.norm();

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

			Eigen::Vector3d lpa(coeff_(0), coeff_(1), coeff_(2));
			Eigen::Vector3d lpb(coeff_(3), coeff_(4), coeff_(5));
			Eigen::Vector3d lp = q_w_curr * point_ + t_w_curr;

			Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
			Eigen::Vector3d de = lpa - lpb;
			Eigen::Vector3d tmp_r = nu / de.norm();
			num_jacobian.col(k) = (tmp_r - r) / eps;
		}
		std::cout << num_jacobian.block<1, 6>(0, 0) << std::endl;
		std::cout << num_jacobian.block<1, 6>(1, 0) << std::endl;
		std::cout << num_jacobian.block<1, 6>(2, 0) << std::endl;
	}

private:
	const Eigen::Vector3d point_;
	const Eigen::VectorXd coeff_;
	const double s_;
};
