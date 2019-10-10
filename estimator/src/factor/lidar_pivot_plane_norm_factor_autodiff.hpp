#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>

class LidarPivotPlaneNormFactor
{
public:
	LidarPivotPlaneNormFactor(Eigen::Vector3d point, Eigen::Vector4d coeff, double s)
    	: point_(point), coeff_(coeff), s_(s){}

	template <typename T>
	bool operator()(const T* const param_a, const T* const param_b, const T* const param_ext, T *residuals) const
	{
		Eigen::Quaternion<T> Q_a(param_a[6], param_a[3], param_a[4], param_a[5]);
		Eigen::Matrix<T, 3, 1> t_a(param_a[0], param_a[1], param_a[2]);
		Eigen::Quaternion<T> Q_b(param_b[6], param_b[3], param_b[4], param_b[5]);
		Eigen::Matrix<T, 3, 1> t_b(param_b[0], param_b[1], param_b[2]);
		Eigen::Quaternion<T> Q_ext(param_ext[6], param_ext[3], param_ext[4], param_ext[5]);
		Eigen::Matrix<T, 3, 1> t_ext(param_ext[0], param_ext[1], param_ext[2]);

		Eigen::Quaternion<T> Q_ext_a = Q_a * Q_ext;
	    Eigen::Matrix<T, 3, 1> t_ext_a = Q_a * t_ext + t_a;
	    Eigen::Quaternion<T> Q_ext_b = Q_b * Q_ext;
	    Eigen::Matrix<T, 3, 1> t_ext_b = Q_b * t_ext + t_b;
	    Eigen::Quaternion<T> Q_ext_ab = Q_ext_a.conjugate() * Q_ext_b;
		Eigen::Matrix<T, 3, 1> t_ext_ab = Q_ext_a.conjugate() * (t_ext_b - t_ext_a);

		Eigen::Matrix<T, 3, 1> w(T(coeff_(0)), T(coeff_(1)), T(coeff_(2)));
		T d = T(coeff_(3));
		Eigen::Matrix<T, 3, 1> p(T(point_(0)), T(point_(1)), T(point_(2)));
		T r = T(w.dot(Q_ext_ab * p + t_ext_ab) + T(d)) * T(s_);
		residuals[0] = r;

		// Eigen::Quaternion<T> Q_a(param_a[6], param_a[3], param_a[4], param_a[5]);
		// Eigen::Matrix<T, 3, 1> t_a(param_a[0], param_a[1], param_a[2]);
		// Eigen::Matrix<T, 4, 4> T_a = Eigen::Matrix<T, 4, 4>::Identity();
		// T_a.template topLeftCorner<3, 3>() = Q_a.toRotationMatrix();
		// T_a.template topRightCorner<3, 1>() = t_a;
		//
		// Eigen::Quaternion<T> Q_b(param_b[6], param_b[3], param_b[4], param_b[5]);
		// Eigen::Matrix<T, 3, 1> t_b(param_b[0], param_b[1], param_b[2]);
		// Eigen::Matrix<T, 4, 4> T_b = Eigen::Matrix<T, 4, 4>::Identity();
		// T_b.template topLeftCorner<3, 3>() = Q_b.toRotationMatrix();
		// T_b.template topRightCorner<3, 1>() = t_b;
		//
		// Eigen::Quaternion<T> Q_ext(param_ext[6], param_ext[3], param_ext[4], param_ext[5]);
		// Eigen::Matrix<T, 3, 1> t_ext(param_ext[0], param_ext[1], param_ext[2]);
		// Eigen::Matrix<T, 4, 4> T_ext = Eigen::Matrix<T, 4, 4>::Identity();
		// T_ext.template topLeftCorner<3, 3>() = Q_ext.toRotationMatrix();
		// T_ext.template topRightCorner<3, 1>() = t_ext;
		//
		// Eigen::Matrix<T, 4, 4> T_ext_a = T_a * T_ext;
		// Eigen::Matrix<T, 4, 4> T_ext_b = T_b * T_ext;
		// Eigen::Matrix<T, 4, 4> T_ext_ab = T_a.inverse() * T_b;
		//
		// Eigen::Matrix<T, 3, 1> w(T(coeff_(0)), T(coeff_(1)), T(coeff_(2)));
		// T d = T(coeff_(3));
		// Eigen::Matrix<T, 3, 1> p(T(point_(0)), T(point_(1)), T(point_(2)));
		// T r = T(w.dot(T_ext_ab.template topLeftCorner<3, 3>() * p + T_ext_ab.template topRightCorner<3, 1>()) + T(d));
		// residuals[0] = r;

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d &point, const Eigen::Vector4d &coeff, const double &s)
	{
		return (new ceres::AutoDiffCostFunction<LidarPivotPlaneNormFactor, 1, 7, 7, 7>(new LidarPivotPlaneNormFactor(point, coeff, s)));
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	const Eigen::Vector3d point_;
	const Eigen::Vector4d coeff_;
	const double s_;
};

// TODO: with jacobian derivation
// bool PivotPointPlaneFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
//   TicToc tic_toc;
//
//   Eigen::Vector3d P_pivot(parameters[0][0], parameters[0][1], parameters[0][2]);
//   Eigen::Quaterniond Q_pivot(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
//
//   Eigen::Vector3d Pi(parameters[1][0], parameters[1][1], parameters[1][2]);
//   Eigen::Quaterniond Qi(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
//
//   Eigen::Vector3d tlb(parameters[2][0], parameters[2][1], parameters[2][2]);
//   Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
//
// //  Eigen::Vector3d tlb = transform_lb_.pos;
// //  Eigen::Quaterniond qlb = transform_lb_.rot;
// ``
//   Eigen::Quaterniond Qlpivot = Q_pivot * qlb.conjugate();
//   Eigen::Vector3d Plpivot = P_pivot - Qlpivot * tlb;
//
//   Eigen::Quaterniond Qli = Qi * qlb.conjugate();
//   Eigen::Vector3d Pli = Pi - Qli * tlb;
//
//   Eigen::Quaterniond Qlpi = Qlpivot.conjugate() * Qli;
//   Eigen::Vector3d Plpi = Qlpivot.conjugate() * (Pli - Plpivot);
//
//   Eigen::Vector3d w(coeff_.x(), coeff_.y(), coeff_.z());
//   double b = coeff_.w();
//
//   double residual = (w.transpose() * (Qlpi * point_ + Plpi) + b);
//
//   double sqrt_info = sqrt_info_static;
//
//   residuals[0] = sqrt_info * residual;
//
//   if (jacobians)
//   {
//     Eigen::Matrix3d Ri = Qi.toRotationMatrix();
//     Eigen::Matrix3d Rp = Q_pivot.toRotationMatrix();
//     Eigen::Matrix3d rlb = qlb.toRotationMatrix();
//     if (jacobians[0])
//     {
//       Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_pivot(jacobians[0]);
//       Eigen::Matrix<double, 1, 6> jaco_pivot;
//
//       jaco_pivot.leftCols<3>() = -w.transpose() * rlb * Rp.transpose();
//       jaco_pivot.rightCols<3>() =
//           w.transpose() * rlb * (SkewSymmetric(Rp.transpose() * Ri * rlb.transpose() * (point_ - tlb))
//               + SkewSymmetric(Rp.transpose() * (Pi - P_pivot)));
//
//       jacobian_pose_pivot.setZero();
//       jacobian_pose_pivot.leftCols<6>() = sqrt_info * jaco_pivot;
//       jacobian_pose_pivot.rightCols<1>().setZero();
//     }
//
//     if (jacobians[1])
//     {
//       Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[1]);
//       Eigen::Matrix<double, 1, 6> jaco_i;
//
//       jaco_i.leftCols<3>() = w.transpose() * rlb * Rp.transpose();
//       jaco_i.rightCols<3>() =
//           w.transpose() * rlb * Rp.transpose() * Ri * (-SkewSymmetric(rlb.transpose() * point_)
//               + SkewSymmetric(rlb.transpose() * tlb));
//
//       jacobian_pose_i.setZero();
//       jacobian_pose_i.leftCols<6>() = sqrt_info * jaco_i;
//       jacobian_pose_i.rightCols<1>().setZero();
//     }
//
//     if (jacobians[2])
//     {
//       Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_ex(jacobians[2]);
//       jacobian_pose_ex.setZero();
//
//       Eigen::Matrix3d I3x3;
//       I3x3.setIdentity();
//
//       Eigen::Matrix3d right_info_mat;
//       right_info_mat.setIdentity();
//       right_info_mat(2, 2) = 1e-6;
//       right_info_mat = Qlpivot.conjugate().normalized() * right_info_mat * Qlpivot.normalized();
//
//       Eigen::Matrix<double, 1, 6> jaco_ex;
//       //  NOTE: planar extrinsic
// //       jaco_ex.leftCols<3>() = w.transpose() * (I3x3 - rlb * Rp.transpose() * Ri * rlb.transpose()) * right_info_mat;
//       jaco_ex.leftCols<3>() = w.transpose() * (I3x3 - rlb * Rp.transpose() * Ri * rlb.transpose());
//       jaco_ex.rightCols<3>() =
//           w.transpose() * rlb * (-SkewSymmetric(Rp.transpose() * Ri * rlb.transpose() * (point_ - tlb))
//               + Rp.transpose() * Ri * SkewSymmetric(rlb.transpose() * (point_ - tlb))
//               - SkewSymmetric(Rp.transpose() * (Pi - P_pivot)));
//
//       jacobian_pose_ex.setZero();
//       jacobian_pose_ex.leftCols<6>() = sqrt_info * jaco_ex;
//       jacobian_pose_ex.rightCols<1>().setZero();
//     }
//   }
//
//   return true;
// }
//
// void PivotPointPlaneFactor::Check(double **parameters) {
//
//   double *res = new double[1];
// //  double **jaco = new double *[1];
//   double **jaco = new double *[3];
//   jaco[0] = new double[1 * 7];
//   jaco[1] = new double[1 * 7];
//   jaco[2] = new double[1 * 7];
//   Evaluate(parameters, res, jaco);
//   DLOG(INFO) << "check begins";
//   DLOG(INFO) << "analytical";
//
//   DLOG(INFO) << *res;
//   DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>>(jaco[0]);
//   DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>>(jaco[1]);
//   DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>>(jaco[2]);
//
//   Eigen::Vector3d P_pivot(parameters[0][0], parameters[0][1], parameters[0][2]);
//   Eigen::Quaterniond Q_pivot(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
//
//   Eigen::Vector3d Pi(parameters[1][0], parameters[1][1], parameters[1][2]);
//   Eigen::Quaterniond Qi(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
//
//   Eigen::Vector3d tlb(parameters[2][0], parameters[2][1], parameters[2][2]);
//   Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
//
// //  Eigen::Vector3d tlb = transform_lb_.pos;
// //  Eigen::Quaterniond qlb = transform_lb_.rot;
//
//   Eigen::Quaterniond Qlpivot = Q_pivot * qlb.conjugate();
//   Eigen::Vector3d Plpivot = P_pivot - Qlpivot * tlb;
//
//   Eigen::Quaterniond Qli = Qi * qlb.conjugate();
//   Eigen::Vector3d Pli = Pi - Qli * tlb;
//
//   Eigen::Quaterniond Qlpi = Qlpivot.conjugate() * Qli;
//   Eigen::Vector3d Plpi = Qlpivot.conjugate() * (Pli - Plpivot);
//
//   Eigen::Vector3d w(coeff_.x(), coeff_.y(), coeff_.z());
//   double b = coeff_.w();
//
//   double sqrt_info = sqrt_info_static;
//
//   double residual = (w.transpose() * (Qlpi * point_ + Plpi) + b);
//
//   residual = sqrt_info * residual;
//   DLOG(INFO) << "num";
//   DLOG(INFO) << residual;
//
//   const double eps = 1e-6;
// //  Eigen::Matrix<double, 1, 6> num_jacobian;
//   Eigen::Matrix<double, 1, 18> num_jacobian;
// //  for (int k = 0; k < 6; k++) {
//   for (int k = 0; k < 18; k++) {
//     Eigen::Vector3d P_pivot(parameters[0][0], parameters[0][1], parameters[0][2]);
//     Eigen::Quaterniond Q_pivot(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
//
//     Eigen::Vector3d Pi(parameters[1][0], parameters[1][1], parameters[1][2]);
//     Eigen::Quaterniond Qi(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
//
//     Eigen::Vector3d tlb(parameters[2][0], parameters[2][1], parameters[2][2]);
//     Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
//
//     int a = k / 3, b = k % 3;
//     Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;
//
//     if (a == 0)
//       P_pivot += delta;
//     else if (a == 1)
//       Q_pivot = Q_pivot * DeltaQ(delta);
//     else if (a == 2)
//       Pi += delta;
//     else if (a == 3)
//       Qi = Qi * DeltaQ(delta);
//     else if (a == 4)
//       tlb += delta;
//     else if (a == 5)
//       qlb = qlb * DeltaQ(delta);
//
//     Eigen::Quaterniond Qlpivot = Q_pivot * qlb.conjugate();
//     Eigen::Vector3d Plpivot = P_pivot - Qlpivot * tlb;
//
//     Eigen::Quaterniond Qli = Qi * qlb.conjugate();
//     Eigen::Vector3d Pli = Pi - Qli * tlb;
//
//     Eigen::Quaterniond Qlpi = Qlpivot.conjugate() * Qli;
//     Eigen::Vector3d Plpi = Qlpivot.conjugate() * (Pli - Plpivot);
//
//     Eigen::Vector3d coeffw(coeff_.x(), coeff_.y(), coeff_.z());
//     double coeffb = coeff_.w();
//
//     double tmp_residual = (coeffw.transpose() * (Qlpi * point_ + Plpi) + coeffb);
//
//     tmp_residual = sqrt_info * tmp_residual;
//
//     num_jacobian(k) = (tmp_residual - residual) / eps;
//   }
//   DLOG(INFO) << std::endl << num_jacobian.block<1, 6>(0, 0);
//   DLOG(INFO) << std::endl << num_jacobian.block<1, 6>(0, 6);
//   DLOG(INFO) << std::endl << num_jacobian.block<1, 6>(0, 12);
// }
