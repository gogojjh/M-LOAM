#include <eigen3/Eigen/Dense>
#include <iostream>

#include "../../estimator/src/estimator/pose.h"
#include "../../estimator/src/factor/lidar_pure_odom_factor.hpp"

int main() {
  Eigen::Vector3d point_;
  point_.setZero();
  Eigen::Vector4d coeffs_;
  coeffs_.setZero();

  Pose pose_pivot, pose_i, pose_ext;

  LidarPureOdomPlaneNormFactor f(point_, coeffs_, 1.0);
  double **param = new double *[3];
  param[0] = new double[1 * 7];
  param[0][0] = pose_pivot.t_(0);
  param[0][1] = pose_pivot.t_(1);
  param[0][2] = pose_pivot.t_(2);
  param[0][3] = pose_pivot.q_.x();
  param[0][4] = pose_pivot.q_.y();
  param[0][5] = pose_pivot.q_.z();
  param[0][6] = pose_pivot.q_.w();

  param[1] = new double[1 * 7];
  param[1][0] = pose_i.t_(0);
  param[1][1] = pose_i.t_(1);
  param[1][2] = pose_i.t_(2);
  param[1][3] = pose_i.q_.x();
  param[1][4] = pose_i.q_.y();
  param[1][5] = pose_i.q_.z();
  param[1][6] = pose_i.q_.w();

  param[2] = new double[1 * 7];
  param[2][0] = pose_ext.t_(0);
  param[2][1] = pose_ext.t_(1);
  param[2][2] = pose_ext.t_(2);
  param[2][3] = pose_ext.q_.x();
  param[2][4] = pose_ext.q_.y();
  param[2][5] = pose_ext.q_.z();
  param[2][6] = pose_ext.q_.w();

  double *res = new double[1];
  double **jaco = new double *[3];
  jaco[0] = new double[1 * 7];
  jaco[1] = new double[1 * 7];
  jaco[2] = new double[1 * 7];
  f.Evaluate(param, res, jaco);

  Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> mat_jacobian_1(
      jaco[0]);
  Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> mat_jacobian_2(
      jaco[1]);
  Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> mat_jacobian_3(
      jaco[2]);
  Eigen::Matrix<double, 3, 7> mat_jacobian;
  mat_jacobian.row(0) = mat_jacobian_1;
  mat_jacobian.row(1) = mat_jacobian_2;
  mat_jacobian.row(2) = mat_jacobian_3;
  Eigen::MatrixXd mat_jaco = mat_jacobian.topLeftCorner<3, 6>();

  delete[] jaco[0];
  delete[] jaco[1];
  delete[] jaco[2];
  delete[] jaco;
  delete[] res;
  delete[] param[0];
  delete[] param[1];
  delete[] param[2];
  delete[] param;
}