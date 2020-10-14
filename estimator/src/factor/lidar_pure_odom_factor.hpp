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

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "../utility/utility.h"

using namespace common;

// pure odom planar factor
class LidarPureOdomPlaneNormFactor : public ceres::SizedCostFunction<1, 7, 7, 7>
{
public:
	LidarPureOdomPlaneNormFactor(const Eigen::Vector3d &point,
								 const Eigen::Vector4d &coeff,
								 const double &sqrt_info = 1.0)
		: point_(point),
		  coeff_(coeff),
		  sqrt_info_(sqrt_info) {}

	// residual = sum(w^(T) * (R * p + t) + d)
	bool Evaluate(double const *const *param, double *residuals, double **jacobians) const
    {
		Eigen::Quaterniond Q_pivot(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_pivot(param[0][0], param[0][1], param[0][2]);
		Eigen::Quaterniond Q_i(param[1][6], param[1][3], param[1][4], param[1][5]);
		Eigen::Vector3d t_i(param[1][0], param[1][1], param[1][2]);
		Eigen::Quaterniond Q_ext(param[2][6], param[2][3], param[2][4], param[2][5]);
		Eigen::Vector3d t_ext(param[2][0], param[2][1], param[2][2]);

		Eigen::Quaterniond Q_pi = Q_pivot.conjugate() * Q_i;
		Eigen::Vector3d t_pi = Q_pivot.conjugate() * (t_i - t_pivot);
		Eigen::Quaterniond Q_ext_pi = Q_pi * Q_ext;
		Eigen::Vector3d t_ext_pi = Q_pi * t_ext + t_pi;

		Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
		const double d = coeff_(3);
		const double r = w.dot(Q_ext_pi * point_ + t_ext_pi) + d;
		residuals[0] = sqrt_info_ * r;

		// jacobians: 3x7
        if (jacobians)
        {
            Eigen::Matrix3d Rp = Q_pivot.toRotationMatrix();
			Eigen::Matrix3d Ri = Q_i.toRotationMatrix();
            Eigen::Matrix3d Rext = Q_ext.toRotationMatrix();
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_pivot(jacobians[0]);

                Eigen::Matrix<double, 1, 6> jaco_pivot;
				jaco_pivot.leftCols<3>() = -w.transpose() * Rp.transpose();
				jaco_pivot.rightCols<3>() = w.transpose() * (Rp.transpose() *
															 Utility::skewSymmetric(Ri * Rext * point_ + Ri * t_ext + t_i - t_pivot));

                jacobian_pose_pivot.leftCols<6>() = sqrt_info_ * jaco_pivot;
                jacobian_pose_pivot.rightCols<1>().setZero();
            }

            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[1]);

                Eigen::Matrix<double, 1, 6> jaco_i;
				jaco_i.leftCols<3>() = w.transpose() * Rp.transpose();
				jaco_i.rightCols<3>() = -w.transpose() * Rp.transpose() *
										Ri * Utility::skewSymmetric(Rext * point_ + t_ext);

                jacobian_pose_i.leftCols<6>() = sqrt_info_ * jaco_i;
                jacobian_pose_i.rightCols<1>().setZero();
            }

            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_ex(jacobians[2]);

				Eigen::Matrix<double, 1, 6> jaco_ex;
				jaco_ex.leftCols<3>() = w.transpose() * Rp.transpose() * Ri;
				jaco_ex.rightCols<3>() = -w.transpose() * Rp.transpose() * Ri * Utility::skewSymmetric(Rext * point_);

                jacobian_pose_ex.leftCols<6>() = sqrt_info_ * jaco_ex;
                jacobian_pose_ex.rightCols<1>().setZero();
            }
        }
        return true;
    }

	// TODO: check if derived jacobian == perturbation on the raw function
    void check(double **param)
    {
        double *res = new double[1];
        double **jaco = new double *[3];
        jaco[0] = new double[1 * 7];
        jaco[1] = new double[1 * 7];
        jaco[2] = new double[1 * 7];
        Evaluate(param, res, jaco);
		std::cout << "[LidarPureOdomPlaneNormFactor] check begins" << std::endl;
        std::cout << "analytical:" << std::endl;

        std::cout << res[0] << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[0]) << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[1]) << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[2]) << std::endl;

		delete[] jaco[0];
		delete[] jaco[1];
		delete[] jaco[2];
		delete[] jaco;
		delete[] res;

		Eigen::Quaterniond Q_pivot(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_pivot(param[0][0], param[0][1], param[0][2]);
		Eigen::Quaterniond Q_i(param[1][6], param[1][3], param[1][4], param[1][5]);
		Eigen::Vector3d t_i(param[1][0], param[1][1], param[1][2]);
		Eigen::Quaterniond Q_ext(param[2][6], param[2][3], param[2][4], param[2][5]);
		Eigen::Vector3d t_ext(param[2][0], param[2][1], param[2][2]);

		Eigen::Quaterniond Q_pi = Q_pivot.conjugate() * Q_i;
		Eigen::Vector3d t_pi = Q_pivot.conjugate() * (t_i - t_pivot);
		Eigen::Quaterniond Q_ext_pi = Q_pi * Q_ext;
		Eigen::Vector3d t_ext_pi = Q_pi * t_ext + t_pi;

		Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
        double d = coeff_(3);
		double r = w.dot(Q_ext_pi * point_ + t_ext_pi) + d;
        r *= sqrt_info_;

        std::cout << "perturbation:" << std::endl;
        std::cout << r << std::endl;

        const double eps = 1e-6;
        Eigen::Matrix<double, 1, 18> num_jacobian;

		// add random perturbation
		for (int k = 0; k < 18; k++)
		{
			Eigen::Quaterniond Q_pivot(param[0][6], param[0][3], param[0][4], param[0][5]);
			Eigen::Vector3d t_pivot(param[0][0], param[0][1], param[0][2]);
			Eigen::Quaterniond Q_i(param[1][6], param[1][3], param[1][4], param[1][5]);
			Eigen::Vector3d t_i(param[1][0], param[1][1], param[1][2]);
			Eigen::Quaterniond Q_ext(param[2][6], param[2][3], param[2][4], param[2][5]);
			Eigen::Vector3d t_ext(param[2][0], param[2][1], param[2][2]);
			int a = k / 3, b = k % 3;
			Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

			if (a == 0)
				t_pivot += delta;
			else if (a == 1)
				Q_pivot = Q_pivot * Utility::deltaQ(delta);
			else if (a == 2)
				t_i += delta;
			else if (a == 3)
				Q_i = Q_i * Utility::deltaQ(delta);
			else if (a == 4)
				t_ext += delta;
			else if (a == 5)
				Q_ext = Q_ext * Utility::deltaQ(delta);

			Eigen::Quaterniond Q_pi = Q_pivot.conjugate() * Q_i;
			Eigen::Vector3d t_pi = Q_pivot.conjugate() * (t_i - t_pivot);
			Eigen::Quaterniond Q_ext_pi = Q_pi * Q_ext;
			Eigen::Vector3d t_ext_pi = Q_pi * t_ext + t_pi;

			Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
	        double d = coeff_(3);
			double tmp_r = w.dot(Q_ext_pi * point_ + t_ext_pi) + d;
	        tmp_r *= sqrt_info_;
            num_jacobian(k) = (tmp_r - r) / eps;
        }
        std::cout << num_jacobian.block<1, 6>(0, 0) << std::endl;
        std::cout << num_jacobian.block<1, 6>(0, 6) << std::endl;
        std::cout << num_jacobian.block<1, 6>(0, 12) << std::endl;
    }

private:
	const Eigen::Vector3d point_;
	const Eigen::Vector4d coeff_;
	const double sqrt_info_;
};

// pure odom planar factor
class LidarPureOdomEdgeFactor : public ceres::SizedCostFunction<1, 7, 7, 7>
{
public:
	LidarPureOdomEdgeFactor(const Eigen::Vector3d &point,
							const Eigen::VectorXd &coeff,
							const double &sqrt_info = 1.0)
		: point_(point),
		  coeff_(coeff),
		  sqrt_info_(sqrt_info) {}

	// residual = sum(w^(T) * (R * p + t) + d)
	bool Evaluate(double const *const *param, double *residuals, double **jacobians) const
    {
		Eigen::Quaterniond Q_pivot(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_pivot(param[0][0], param[0][1], param[0][2]);
		Eigen::Quaterniond Q_i(param[1][6], param[1][3], param[1][4], param[1][5]);
		Eigen::Vector3d t_i(param[1][0], param[1][1], param[1][2]);
		Eigen::Quaterniond Q_ext(param[2][6], param[2][3], param[2][4], param[2][5]);
		Eigen::Vector3d t_ext(param[2][0], param[2][1], param[2][2]);

		Eigen::Quaterniond Q_pi = Q_pivot.conjugate() * Q_i;
		Eigen::Vector3d t_pi = Q_pivot.conjugate() * (t_i - t_pivot);
		Eigen::Quaterniond Q_ext_pi = Q_pi * Q_ext;
		Eigen::Vector3d t_ext_pi = Q_pi * t_ext + t_pi;

		Eigen::Vector3d lpa(coeff_(0), coeff_(1), coeff_(2));
		Eigen::Vector3d lpb(coeff_(3), coeff_(4), coeff_(5));
		Eigen::Vector3d lp = Q_ext_pi * point_ + t_ext_pi;

        Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
        Eigen::Vector3d de = lpa - lpb;
        residuals[0] = sqrt_info_ * nu.norm() / de.norm();		

		// jacobians: 1x21
        if (jacobians)
        {
            Eigen::Matrix3d Rp = Q_pivot.toRotationMatrix();
			Eigen::Matrix3d Ri = Q_i.toRotationMatrix();
            Eigen::Matrix3d Rext = Q_ext.toRotationMatrix();

			Eigen::Matrix<double, 1, 3> eta = 1.0 / de.norm() * nu.normalized().transpose();
			Eigen::Vector3d ba = lp - lpa;
			Eigen::Vector3d bb = lp - lpb;

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_pivot(jacobians[0]);

                Eigen::Matrix<double, 1, 6> jaco_pivot;
				jaco_pivot.leftCols<3>() = - eta * Utility::skewSymmetric(ba - bb) * Rp.transpose();
				jaco_pivot.rightCols<3>() = eta * Utility::skewSymmetric(ba - bb) * 
					Utility::skewSymmetric(Rp.transpose() * (Ri * Rext * point_ + Ri * t_ext + t_i - t_pivot));

				jacobian_pose_pivot.setZero();
                jacobian_pose_pivot.leftCols<6>() = sqrt_info_ * jaco_pivot;
            }

            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[1]);

                Eigen::Matrix<double, 1, 6> jaco_i;
				jaco_i.leftCols<3>() = eta * Utility::skewSymmetric(ba - bb) * Rp.transpose();
				jaco_i.rightCols<3>() = - eta * Utility::skewSymmetric(ba - bb) * Rp.transpose() * Ri * 
					Utility::skewSymmetric(Rext * point_ + t_ext);

				jacobian_pose_i.setZero();
                jacobian_pose_i.leftCols<6>() = sqrt_info_ * jaco_i;
            }

            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_ex(jacobians[2]);

				Eigen::Matrix<double, 1, 6> jaco_ex;
				jaco_ex.leftCols<3>() = eta * Utility::skewSymmetric(ba - bb) * Rp.transpose() * Ri;
				jaco_ex.rightCols<3>() = - eta * Utility::skewSymmetric(ba - bb) * Rp.transpose() * Ri * 
					(Rext * Utility::skewSymmetric(point_) + Utility::skewSymmetric(t_ext));

				jacobian_pose_ex.setZero();
                jacobian_pose_ex.leftCols<6>() = sqrt_info_ * jaco_ex;
            }
        }
        return true;
    }

	// TODO: check if derived jacobian == perturbation on the raw function
    void check(double **param)
    {
        double *res = new double[1];
        double **jaco = new double *[3];
        jaco[0] = new double[1 * 7];
        jaco[1] = new double[1 * 7];
        jaco[2] = new double[1 * 7];
        Evaluate(param, res, jaco);
		std::cout << "[LidarPureOdomEdgeFactor] check begins" << std::endl;
        std::cout << "analytical:" << std::endl;

        std::cout << res[0] << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[0]) << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[1]) << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[2]) << std::endl;

		delete[] jaco[0];
		delete[] jaco[1];
		delete[] jaco[2];
		delete[] jaco;
		delete[] res;

		Eigen::Quaterniond Q_pivot(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_pivot(param[0][0], param[0][1], param[0][2]);
		Eigen::Quaterniond Q_i(param[1][6], param[1][3], param[1][4], param[1][5]);
		Eigen::Vector3d t_i(param[1][0], param[1][1], param[1][2]);
		Eigen::Quaterniond Q_ext(param[2][6], param[2][3], param[2][4], param[2][5]);
		Eigen::Vector3d t_ext(param[2][0], param[2][1], param[2][2]);

		Eigen::Quaterniond Q_pi = Q_pivot.conjugate() * Q_i;
		Eigen::Vector3d t_pi = Q_pivot.conjugate() * (t_i - t_pivot);
		Eigen::Quaterniond Q_ext_pi = Q_pi * Q_ext;
		Eigen::Vector3d t_ext_pi = Q_pi * t_ext + t_pi;

		Eigen::Vector3d lpa(coeff_(0), coeff_(1), coeff_(2));
		Eigen::Vector3d lpb(coeff_(3), coeff_(4), coeff_(5));
		Eigen::Vector3d lp = Q_ext_pi * point_ + t_ext_pi;

        Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
        Eigen::Vector3d de = lpa - lpb;
        double r = sqrt_info_ * nu.norm() / de.norm();		

        std::cout << "perturbation:" << std::endl;
        std::cout << r << std::endl;

        const double eps = 1e-6;
        Eigen::Matrix<double, 1, 18> num_jacobian;

		// add random perturbation
		for (int k = 0; k < 18; k++)
		{
			Eigen::Quaterniond Q_pivot(param[0][6], param[0][3], param[0][4], param[0][5]);
			Eigen::Vector3d t_pivot(param[0][0], param[0][1], param[0][2]);
			Eigen::Quaterniond Q_i(param[1][6], param[1][3], param[1][4], param[1][5]);
			Eigen::Vector3d t_i(param[1][0], param[1][1], param[1][2]);
			Eigen::Quaterniond Q_ext(param[2][6], param[2][3], param[2][4], param[2][5]);
			Eigen::Vector3d t_ext(param[2][0], param[2][1], param[2][2]);
			int a = k / 3, b = k % 3;
			Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

			if (a == 0)
				t_pivot += delta;
			else if (a == 1)
				Q_pivot = Q_pivot * Utility::deltaQ(delta);
			else if (a == 2)
				t_i += delta;
			else if (a == 3)
				Q_i = Q_i * Utility::deltaQ(delta);
			else if (a == 4)
				t_ext += delta;
			else if (a == 5)
				Q_ext = Q_ext * Utility::deltaQ(delta);

			Eigen::Quaterniond Q_pi = Q_pivot.conjugate() * Q_i;
			Eigen::Vector3d t_pi = Q_pivot.conjugate() * (t_i - t_pivot);
			Eigen::Quaterniond Q_ext_pi = Q_pi * Q_ext;
			Eigen::Vector3d t_ext_pi = Q_pi * t_ext + t_pi;

			Eigen::Vector3d lpa(coeff_(0), coeff_(1), coeff_(2));
			Eigen::Vector3d lpb(coeff_(3), coeff_(4), coeff_(5));
			Eigen::Vector3d lp = Q_ext_pi * point_ + t_ext_pi;

			Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
			Eigen::Vector3d de = lpa - lpb;
			double tmp_r = sqrt_info_ * nu.norm() / de.norm();
            num_jacobian(k) = (tmp_r - r) / eps;
        }
        std::cout << num_jacobian.block<1, 6>(0, 0) << std::endl;
        std::cout << num_jacobian.block<1, 6>(0, 6) << std::endl;
        std::cout << num_jacobian.block<1, 6>(0, 12) << std::endl;
    }

private:
	const Eigen::Vector3d point_;
	const Eigen::VectorXd coeff_;
	const double sqrt_info_;
};


// pure odom edge factor using auto diff
class LidarPureOdomEdgeFactorAuto
{
public:
	LidarPureOdomEdgeFactorAuto(const Eigen::Vector3d &point,
							    const Eigen::VectorXd &coeff,
							    const double &sqrt_info = 1.0)
		: point_(point),
		  coeff_(coeff),
		  sqrt_info_(sqrt_info) {}

	template <typename T>
	bool operator()(const T *param1, const T *param2, const T *param3, T *residuals) const
	{
		Eigen::Quaternion<T> Q_pivot(param1[6], param1[3], param1[4], param1[5]);
		Eigen::Matrix<T, 3, 1> t_pivot(param1[0], param1[1], param1[2]);
		Eigen::Quaternion<T> Q_i(param2[6], param2[3], param2[4], param2[5]);
		Eigen::Matrix<T, 3, 1> t_i(param2[0], param2[1], param2[2]);
		Eigen::Quaternion<T> Q_ext(param3[6], param3[3], param3[4], param3[5]);
		Eigen::Matrix<T, 3, 1> t_ext(param3[0], param3[1], param3[2]);

		Eigen::Quaternion<T> Q_pi = Q_pivot.conjugate() * Q_i;
		Eigen::Matrix<T, 3, 1> t_pi = Q_pivot.conjugate() * (t_i - t_pivot);
		Eigen::Quaternion<T> Q_ext_pi = Q_pi * Q_ext;
		Eigen::Matrix<T, 3, 1> t_ext_pi = Q_pi * t_ext + t_pi;

		Eigen::Matrix<T, 3, 1> cp(T(point_.x()), T(point_.y()), T(point_.z()));
		Eigen::Matrix<T, 3, 1> lpa(T(coeff_(0)), T(coeff_(1)), T(coeff_(2)));
		Eigen::Matrix<T, 3, 1> lpb(T(coeff_(3)), T(coeff_(4)), T(coeff_(5)));
		Eigen::Matrix<T, 3, 1> lp = Q_ext_pi * cp + t_ext_pi;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;
		// residuals[0] = T(sqrt_info_) * nu.x() / de.norm();
		// residuals[1] = T(sqrt_info_) * nu.y() / de.norm();
		// residuals[2] = T(sqrt_info_) * nu.z() / de.norm();
		residuals[0] = T(sqrt_info_) * nu.norm() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d &point,
									   const Eigen::VectorXd &coeff,
									   const double sqrt_info)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPureOdomEdgeFactorAuto, 1, 7, 7, 7>(
			new LidarPureOdomEdgeFactorAuto(point, coeff, sqrt_info)));
	}

private: 
	const Eigen::Vector3d point_;
	const Eigen::VectorXd coeff_;
	const double sqrt_info_;
};

//
