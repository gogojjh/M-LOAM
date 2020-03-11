#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"

// calculate distrance from point to plane (using normal)
class LidarScanPlaneNormFactor : public ceres::SizedCostFunction<1, 7>
{
public:
	LidarScanPlaneNormFactor(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, const double &s)
		: point_(point), coeff_(coeff), s_(s) {}

	bool Evaluate(double const *const *param, double *residuals, double **jacobians) const
	{
		Eigen::Quaterniond q_last_curr(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_last_curr(param[0][0], param[0][1], param[0][2]);
		// q_last_curr = Eigen::Quaterniond::Identity().slerp(s_, q_last_curr);
		// t_last_curr = s_ * t_last_curr;

		Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
		double d = coeff_(3);
		double a = w.dot(q_last_curr * point_ + t_last_curr) + d;
		residuals[0] = a;

		if (jacobians)
		{
			Eigen::Matrix3d R = q_last_curr.toRotationMatrix();
			if (jacobians[0])
			{
				Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose(jacobians[0]);
				Eigen::Matrix<double, 1, 6> jaco; // [dy/dt, dy/dR, 1]
				jaco.setZero();
				jaco.leftCols<3>() = w.transpose();
				jaco.rightCols<3>() = -w.transpose() * R * Utility::skewSymmetric(point_);

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
		jaco[0] = new double[1 * 7];
		Evaluate(param, res, jaco);
		std::cout << "[LidarScanPlaneNormFactor] check begins" << std::endl;
		std::cout << "analytical:" << std::endl;
        std::cout << res[0] << " " << res[1] << " " << res[2] << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> >(jaco[0]) << std::endl;

		Eigen::Quaterniond q_last_curr(param[0][6], param[0][3], param[0][4], param[0][5]);
		Eigen::Vector3d t_last_curr(param[0][0], param[0][1], param[0][2]);
		q_last_curr = Eigen::Quaterniond::Identity().slerp(s_, q_last_curr);
		t_last_curr = s_ * t_last_curr;

		Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
		double d = coeff_(3);
		double a = w.dot(q_last_curr * point_ + t_last_curr) + d;
		double r = a;

        std::cout << "perturbation:" << std::endl;
        std::cout << r << std::endl;

        const double eps = 1e-6;
        Eigen::Matrix<double, 1, 6> num_jacobian;

		// add random perturbation
		for (int k = 0; k < 6; k++)
		{
			Eigen::Quaterniond q_last_curr(param[0][6], param[0][3], param[0][4], param[0][5]);
			Eigen::Vector3d t_last_curr(param[0][0], param[0][1], param[0][2]);
			q_last_curr = Eigen::Quaterniond::Identity().slerp(s_, q_last_curr);
			t_last_curr = s_ * t_last_curr;

			int a = k / 3, b = k % 3;
			Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;
			if (a == 0)
				t_last_curr += delta;
			else if (a == 1)
				q_last_curr = q_last_curr * Utility::deltaQ(delta);

			Eigen::Vector3d w(coeff_(0), coeff_(1), coeff_(2));
	        double d = coeff_(3);
			double v = w.dot(q_last_curr * point_ + t_last_curr) + d;
			double tmp_r = v;
            num_jacobian(0, k) = (tmp_r - r) / eps;
        }
        std::cout << num_jacobian << std::endl;
    }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
private:
	Eigen::Vector3d point_;
	Eigen::Vector4d coeff_;
	double s_;
};
