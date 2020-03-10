#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"

// calculate distrance from point to plane (using normal)
struct LidarScanPlaneNormFactor 
{
	LidarScanPlaneNormFactor(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, const double &s)
		: point_(point), coeff_(coeff), s_(s) {}

	template<typename T>
	bool operator()(const T *param, T *residuals) const
	{
		Eigen::Quaternion<T> q_last_curr(param[6], param[3], param[4], param[5]);
		Eigen::Matrix<T, 3, 1> t_last_curr(param[0], param[1], param[2]);
		Eigen::Quaternion<T> q_identity(T(1), T(0), T(0), T(0));
		q_last_curr = q_identity.slerp(T(s_), q_last_curr);
		t_last_curr = T(s_) * t_last_curr;

		Eigen::Matrix<T, 3, 1> w(T(coeff_(0)), T(coeff_(1)), T(coeff_(2)));
		Eigen::Matrix<T, 3, 1> cp(T(point_(0)), T(point_(1)), T(point_(2)));
		T d = T(coeff_(3));
		T a = w.dot(q_last_curr * cp + t_last_curr) + d;
		Eigen::Map<Eigen::Matrix<T, 3, 1> > r(residuals);
		r = a * w;
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, const double &s)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarScanPlaneNormFactor, 3, 7>(new LidarScanPlaneNormFactor(point, coeff, s)));
	}

	Eigen::Vector3d point_;
	Eigen::Vector4d coeff_;
	double s_;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
