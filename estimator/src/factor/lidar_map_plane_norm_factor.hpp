#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

// calculate distrance from point to plane (using normal)
struct LidarMapPlaneNormFactor
{
	LidarMapPlaneNormFactor(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, const double &s, const double &sqrt_info_static = 1.0)
		: point_(point), coeff_(coeff), s_(s), sqrt_info_static_(sqrt_info_static){}

	template <typename T>
	bool operator()(const T *param, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr(param[6], param[3], param[4], param[5]);
		Eigen::Matrix<T, 3, 1> t_w_curr(param[0], param[1], param[2]);

		Eigen::Matrix<T, 3, 1> w(T(coeff_(0)), T(coeff_(1)), T(coeff_(2)));
		Eigen::Matrix<T, 3, 1> cp(T(point_(0)), T(point_(1)), T(point_(2)));
		T d = T(coeff_(3));
		T r = (w.dot(q_w_curr * cp + t_w_curr) + d) * T(s_);
		residual[0] = T(sqrt_info_static_) * r;
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, const double &s, const double &sqrt_info_static)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarMapPlaneNormFactor, 1, 7>(new LidarMapPlaneNormFactor(point, coeff, s, sqrt_info_static)));
	}

	Eigen::Vector3d point_;
	Eigen::Vector4d coeff_;
	double s_;
	double sqrt_info_static_;
};
