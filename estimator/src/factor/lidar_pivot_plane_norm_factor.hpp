#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "common/algos/math.hpp"

using namespace common;

static double sqrt_info_static = 1.0;

class LidarPivotPlaneNormFactor : public ceres::SizedCostFunction<1, 7, 7, 7>
{
public:
	LidarPivotPlaneNormFactor(Eigen::Vector3d point, Eigen::Vector4d coeff, double s)
    	: point_(point), coeff_(coeff), s_(s){}

	// TODO:
	// residual = sum(w^(T) * (R * p + t) + d)
    bool Evaluate(double const *const *param, double *residuals, double **jacobians) const
    {
		Eigen::Quaterniond Q_pivot(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_pivot(param[0][0], param[0][1], param[0][2]);
		Eigen::Quaterniond Q_i(param[1][6], param[1][3], param[1][4], param[1][5]);
		Eigen::Vector3d t_i(param[1][0], param[1][1], param[1][2]);
		Eigen::Quaterniond Q_ext(param[2][6], param[2][3], param[2][4], param[2][5]);
		Eigen::Vector3d t_ext(param[2][0], param[2][1], param[2][2]);

		Eigen::Quaterniond Q_ext_pivot = Q_pivot * Q_ext;
		Eigen::Vector3d t_ext_pivot = Q_pivot * t_ext + t_pivot;
		Eigen::Quaterniond Q_ext_i = Q_i * Q_ext;
		Eigen::Vector3d t_ext_i = Q_i * t_ext + t_i;
		Eigen::Quaterniond Q_ext_pi = Q_ext_pivot.conjugate() * Q_ext_i;
		Eigen::Vector3d t_ext_pi = Q_ext_pivot.conjugate() * (t_ext_i - t_ext_pivot);

        Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
        double d = coeff_(3);
		double r = (w.dot(Q_ext_pi * point_ + t_ext_pi) + d) * s_;
        double sqrt_info = sqrt_info_static;
        residuals[0] = sqrt_info * r;

		// TODO: jacobians derivation
        if (jacobians)
        {
            Eigen::Matrix3d Rp = Q_pivot.toRotationMatrix();
			Eigen::Matrix3d Ri = Q_i.toRotationMatrix();
            Eigen::Matrix3d Rext = Q_ext.toRotationMatrix();
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_pivot(jacobians[0]);
                Eigen::Matrix<double, 1, 6> jaco_pivot;

                jaco_pivot.leftCols<3>() = -w.transpose() * Rext * Rp.transpose();
                jaco_pivot.rightCols<3>() = w.transpose() * Rext * (
						SkewSymmetric(Rp.transpose() * Ri * Rext.transpose() * (point_ - t_ext)) +
						SkewSymmetric(Rp.transpose() * (t_i - t_pivot)));

                jacobian_pose_pivot.setZero();
                jacobian_pose_pivot.leftCols<6>() = sqrt_info * jaco_pivot;
                jacobian_pose_pivot.rightCols<1>().setZero();
            }

            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[1]);
                Eigen::Matrix<double, 1, 6> jaco_i;

                jaco_i.leftCols<3>() = w.transpose() * Rext * Rp.transpose();
                jaco_i.rightCols<3>() =
					w.transpose() * Rext * Rp.transpose() * Ri * (-SkewSymmetric(Rext.transpose() * point_)
					  	+ SkewSymmetric(Rext.transpose() * t_ext));

                jacobian_pose_i.setZero();
                jacobian_pose_i.leftCols<6>() = sqrt_info * jaco_i;
                jacobian_pose_i.rightCols<1>().setZero();
            }

            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_ex(jacobians[2]);
                jacobian_pose_ex.setZero();

                Eigen::Matrix3d I3x3;
                I3x3.setIdentity();

                Eigen::Matrix3d right_info_mat;
                right_info_mat.setIdentity();
                right_info_mat(2, 2) = 1e-6;
                right_info_mat = Q_ext_pivot.conjugate().normalized() * right_info_mat * Q_ext_pivot.normalized();

                Eigen::Matrix<double, 1, 6> jaco_ex;
                //  NOTE: planar extrinsic
                //       jaco_ex.leftCols<3>() = w.transpose() * (I3x3 - Rext * Rp.transpose() * Ri * Rext.transpose()) * right_info_mat;
                jaco_ex.leftCols<3>() = w.transpose() * (I3x3 - Rext * Rp.transpose() * Ri * Rext.transpose());
                jaco_ex.rightCols<3>() =
					w.transpose() * Rext * (-SkewSymmetric(Rp.transpose() * Ri * Rext.transpose() * (point_ - t_ext))
						  + Rp.transpose() * Ri * SkewSymmetric(Rext.transpose() * (point_ - t_ext))
						  - SkewSymmetric(Rp.transpose() * (t_i - t_pivot)));

                jacobian_pose_ex.setZero();
                jacobian_pose_ex.leftCols<6>() = sqrt_info * jaco_ex;
                jacobian_pose_ex.rightCols<1>().setZero();
            }
        }
        return true;
    }

    void Check(double **param)
    {
        double *res = new double[1];
        //  double **jaco = new double *[1];
        double **jaco = new double *[3];
        jaco[0] = new double[1 * 7];
        jaco[1] = new double[1 * 7];
        jaco[2] = new double[1 * 7];
        Evaluate(param, res, jaco);
        std::cout << "check begins" << std::endl;
        std::cout << "analytical" << std::endl;

        std::cout << *res << std::endl;
        std::cout << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[0]) << std::endl;
        std::cout << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[1]) << std::endl;
        std::cout << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[2]) << std::endl;

		Eigen::Quaterniond Q_pivot(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_pivot(param[0][0], param[0][1], param[0][2]);
		Eigen::Quaterniond Q_i(param[1][6], param[1][3], param[1][4], param[1][5]);
		Eigen::Vector3d t_i(param[1][0], param[1][1], param[1][2]);
		Eigen::Quaterniond Q_ext(param[2][6], param[2][3], param[2][4], param[2][5]);
		Eigen::Vector3d t_ext(param[2][0], param[2][1], param[2][2]);

		Eigen::Quaterniond Q_ext_pivot = Q_pivot * Q_ext;
		Eigen::Vector3d t_ext_pivot = Q_pivot * t_ext + t_pivot;
		Eigen::Quaterniond Q_ext_i = Q_i * Q_ext;
		Eigen::Vector3d t_ext_i = Q_i * t_ext + t_i;
		Eigen::Quaterniond Q_ext_pi = Q_ext_pivot.conjugate() * Q_ext_i;
		Eigen::Vector3d t_ext_pi = Q_ext_pivot.conjugate() * (t_ext_i - t_ext_pivot);

		Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
        double d = coeff_(3);
		double r = w.dot(Q_ext_pi * point_ + t_ext_pi) + d;
        double sqrt_info = sqrt_info_static;
        r *= sqrt_info;

        std::cout << "num" << std::endl;
        std::cout << r << std::endl;

        const double eps = 1e-6;
        //  Eigen::Matrix<double, 1, 6> num_jacobian;
        Eigen::Matrix<double, 1, 18> num_jacobian;
        //  for (int k = 0; k < 6; k++) {
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
                Q_pivot = Q_pivot * DeltaQ(delta);
            else if (a == 2)
                t_i += delta;
            else if (a == 3)
                Q_i = Q_i * DeltaQ(delta);
            else if (a == 4)
                t_ext += delta;
            else if (a == 5)
                Q_ext = Q_ext * DeltaQ(delta);

			Eigen::Quaterniond Q_ext_pivot = Q_pivot * Q_ext;
			Eigen::Vector3d t_ext_pivot = Q_pivot * t_ext + t_pivot;
			Eigen::Quaterniond Q_ext_i = Q_i * Q_ext;
			Eigen::Vector3d t_ext_i = Q_i * t_ext + t_i;
			Eigen::Quaterniond Q_ext_pi = Q_ext_pivot.conjugate() * Q_ext_i;
			Eigen::Vector3d t_ext_pi = Q_ext_pivot.conjugate() * (t_ext_i - t_ext_pivot);

			Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
	        double d = coeff_(3);
			double tmp_r = w.dot(Q_ext_pi * point_ + t_ext_pi) + d;
	        tmp_r *= sqrt_info;
            num_jacobian(k) = (tmp_r - r) / eps;
        }
        std::cout << std::endl << num_jacobian.block<1, 6>(0, 0) << std::endl;
        std::cout << std::endl << num_jacobian.block<1, 6>(0, 6) << std::endl;
        std::cout << std::endl << num_jacobian.block<1, 6>(0, 12) << std::endl;
    }

private:
	const Eigen::Vector3d point_;
	const Eigen::Vector4d coeff_;
	const double s_;
};


//
