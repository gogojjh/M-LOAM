#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

class LidarPivotTargetPlaneNormFactor
{
public:
	LidarPivotTargetPlaneNormFactor(Eigen::Vector3d point, Eigen::Vector4d coeff, double s)
    	: point_(point), coeff_(coeff), s_(s){}

	// jacobian derivation
	template <typename T>
	bool operator()(const T* const param_a, const T* const param_b, const T* const param_ext, T *residuals) const
	{
		Eigen::Quaternion<T> Q_pivot(param_a[6], param_a[3], param_a[4], param_a[5]);
		Eigen::Matrix<T, 3, 1> t_pivot(param_a[0], param_a[1], param_a[2]);
		Eigen::Quaternion<T> Q_i(param_b[6], param_b[3], param_b[4], param_b[5]);
		Eigen::Matrix<T, 3, 1> t_i(param_b[0], param_b[1], param_b[2]);
		Eigen::Quaternion<T> Q_ext(param_ext[6], param_ext[3], param_ext[4], param_ext[5]);
		Eigen::Matrix<T, 3, 1> t_ext(param_ext[0], param_ext[1], param_ext[2]);

		Eigen::Quaternion<T> Q_pi = Q_pivot.conjugate() * Q_i;
		Eigen::Matrix<T, 3, 1> t_pi = Q_pivot.conjugate() * (t_i - t_pivot);
		Eigen::Quaternion<T> Q_p_ext_i = Q_pi * Q_ext;
		Eigen::Matrix<T, 3, 1> t_p_ext_i = Q_pi * t_ext + t_pi;

		Eigen::Matrix<T, 3, 1> w(T(coeff_(0)), T(coeff_(1)), T(coeff_(2)));
		T d = T(coeff_(3));
		Eigen::Matrix<T, 3, 1> p(T(point_(0)), T(point_(1)), T(point_(2)));
		T r = T(w.dot(Q_p_ext_i * p + t_p_ext_i) + T(d)) * T(s_);
		residuals[0] = r;
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, const double &s)
	{
		return (new ceres::AutoDiffCostFunction<LidarPivotTargetPlaneNormFactor, 1, 7, 7, 7>(new LidarPivotTargetPlaneNormFactor(point, coeff, s)));
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	const Eigen::Vector3d point_;
	const Eigen::Vector4d coeff_;
	const double s_;
};
