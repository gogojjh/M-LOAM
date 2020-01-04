#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "../utility/utility.h"

class LidarPivotTargetPlaneNormFactor: public ceres::SizedCostFunction<1, 7>
{
public:
	LidarPivotTargetPlaneNormFactor(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, double sqrt_info = 1.0)
    	: point_(point), coeff_(coeff), sqrt_info_(sqrt_info){}

	// TODO: jacobian derivation
	bool Evaluate(double const *const *param, double *residuals, double **jacobians) const
	{
		Eigen::Quaterniond Q_ext(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_ext(param[0][0], param[0][1], param[0][2]);

		Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
		double d = coeff_(3);
		double r = w.dot(Q_ext * point_ + t_ext) + d;
		residuals[0] = sqrt_info_ * r;

		if (jacobians)
		{
			Eigen::Matrix3d Rext = Q_ext.toRotationMatrix();
			if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_ext(jacobians[0]);
                Eigen::Matrix<double, 1, 6> jaco_ext;

                jaco_ext.leftCols<3>() = w.transpose();
				jaco_ext.rightCols<3>() = -w.transpose() * Rext * Utility::skewSymmetric(point_);

                jacobian_pose_ext.setZero();
                jacobian_pose_ext.leftCols<6>() = sqrt_info_ * jaco_ext;
                jacobian_pose_ext.rightCols<1>().setZero();
            }
		}
        return true;
	}

	// TODO: check the jacobian derivation
	void check(double **param)
    {
		double *res = new double[1];
        //  double **jaco = new double *[1];
        double **jaco = new double *[3];
        jaco[0] = new double[1 * 7];
        Evaluate(param, res, jaco);
        std::cout << "[LidarPivotTargetPlaneNormFactor] check begins" << std::endl;
        std::cout << "analytical:" << std::endl;

        std::cout << *res << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> >(jaco[0]) << std::endl;

		Eigen::Quaterniond Q_ext(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_ext(param[0][0], param[0][1], param[0][2]);

		Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
		double d = coeff_(3);
		double r = w.dot(Q_ext * point_ + t_ext) + d;
        r *= sqrt_info_;

        std::cout << "perturbation:" << std::endl;
        std::cout << r << std::endl;

        const double eps = 1e-6;
        Eigen::Matrix<double, 1, 6> num_jacobian;

		// add random perturbation
        for (int k = 0; k < 6; k++)
        {
			Eigen::Quaterniond Q_ext(param[0][6], param[0][3], param[0][4], param[0][5]);
			Eigen::Vector3d t_ext(param[0][0], param[0][1], param[0][2]);
            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

         	if (a == 0)
                t_ext += delta;
            else if (a == 1)
                Q_ext = Q_ext * Utility::deltaQ(delta);

			Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
			double d = coeff_(3);
			double tmp_r = w.dot(Q_ext * point_ + t_ext) + d;
	        tmp_r *= sqrt_info_;
            num_jacobian(k) = (tmp_r - r) / eps;
        }
        std::cout << num_jacobian.block<1, 6>(0, 0) << std::endl;
	}

private:
	const Eigen::Vector3d point_;
	const Eigen::Vector4d coeff_;
	const double sqrt_info_;
};
