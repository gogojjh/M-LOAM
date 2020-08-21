// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "impl_loss_function.hpp"

// Data generated using the following octave code.
//   randn('seed', 23497);
//   m = 0.3;
//   c = 0.1;
//   x=[0:0.075:5];
//   y = exp(m * x + c);
//   noise = randn(size(x)) * 0.2;
//   outlier_noise = rand(size(x)) < 0.05;
//   y_observed = y + noise + outlier_noise;
//   data = [x', y_observed'];

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

const int kNumObservations = 10;
const double data[] = {
    0.000000e+00, 1.133898e+00,
    7.500000e-02, 1.334902e+00,
    1.050000e+00, 1.402653e+00,
    1.125000e+00, 1.713141e+00,
    1.200000e+00, 1.527021e+00,
    1.275000e+00, 1.702632e+00,
    1.350000e+00, 1.423899e+00,
    1.425000e+00, 5.543078e+00, // Outlier point
    2.100000e+00, 2.169119e+00,
    2.175000e+00, 2.061745e+00};

double *param = new double[2];

template <typename T>
inline void CRSMatrix2EigenMatrix(const ceres::CRSMatrix &crs_matrix, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &eigen_matrix)
{
  eigen_matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(crs_matrix.num_rows, crs_matrix.num_cols);
  for (auto row = 0; row < crs_matrix.num_rows; row++)
  {
    int start = crs_matrix.rows[row];
    int end = crs_matrix.rows[row + 1] - 1;
    for (auto i = start; i <= end; i++)
    {
      int col = crs_matrix.cols[i];
      eigen_matrix(row, col) = T(crs_matrix.values[i]);
    }
  }
}

struct ExponentialResidual {
  ExponentialResidual(double x, double y)
      : x_(x), y_(y) {}

  template <typename T> bool operator()(const T* const param, T* residual) const 
  {
    residual[0] = y_ - exp(param[0] * x_ + param[1]);
    return true;
  }

 private:
  const double x_;
  const double y_;
};

using namespace ceres::internal;
using namespace ceres;
class LoggingCallback : public IterationCallback
{
public:
  explicit LoggingCallback(Problem *problem, double *param)
      : problem_(problem), param_(param) {}

  ~LoggingCallback() {}

  virtual CallbackReturnType operator()(const IterationSummary &summary)
  {
    std::cout << "iteration: " << summary.iteration << ": "
              << param_[0] << " " << param_[1] << std::endl;
    double cost;
    ceres::CRSMatrix jaco;
    problem_->Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, &jaco);
    Eigen::MatrixXd mat_J;
    CRSMatrix2EigenMatrix(jaco, mat_J);
    std::cout << "jacobian: \n" << mat_J.transpose() << std::endl;
    Eigen::MatrixXd mat_H = mat_J.transpose() * mat_J;
    std::cout << "covariance: \n" << mat_H.inverse() << std::endl;
    return SOLVER_CONTINUE;
  }

private:
  double *param_;
  Problem *problem_;
};

int main(int argc, char** argv) 
{
  google::InitGoogleLogging(argv[0]);

  // std::vector<double> data;
  // FILE *file;
  // file = std::fopen("/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_test/data/data_curve_fitting.txt", "r");
  // double x, y;
  // char ch;
  // std::string str;
  // fscanf(file, "%s", &str);
  // while (fscanf(file, "%lf %c %lf", &x, &ch, &y) != EOF)
  // {
  //   data.push_back(x);
  //   data.push_back(y);
  // }
  // std::fclose(file);
  // size_t kNumObservations = floor(data.size() / 2);

  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
  // ceres::LossFunction *loss_function = new ceres::GemanMcClureLoss(1);

  Problem problem;
  problem.AddParameterBlock(param, 2, NULL);
  for (int i = 0; i < kNumObservations; ++i) {
    CostFunction* cost_function =
        new AutoDiffCostFunction<ExponentialResidual, 1, 2>(
            new ExponentialResidual(data[2 * i], data[2 * i + 1]));
    problem.AddResidualBlock(cost_function,
                             loss_function,
                             param);
  }

  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.check_gradients = true;
  options.gradient_check_relative_precision = 1e-4;
  options.update_state_every_iteration = true;
  // options.max_num_iterations = 50;
  IterationCallback *logging_callback = new LoggingCallback(&problem, param);
  options.callbacks.push_back(logging_callback);

  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  // ceres covariance calculation
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);
  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  covariance_blocks.push_back(std::make_pair(param, param));
  covariance.Compute(covariance_blocks, &problem);
  double *covariance_xx = new double[2 * 2];
  covariance.GetCovarianceBlock(param, param, covariance_xx);
  std::cout << "covariance: " << std::endl
            << covariance_xx[0] << " " << covariance_xx[1] << std::endl
            << covariance_xx[2] << " " << covariance_xx[3] << std::endl;

  // self-making covariance calculation
  double cost;
  ceres::CRSMatrix jaco;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, &jaco);
  Eigen::MatrixXd mat_J;
  CRSMatrix2EigenMatrix(jaco, mat_J);
  Eigen::MatrixXd mat_Jt = mat_J.transpose();
  Eigen::MatrixXd mat_H = mat_Jt * mat_J;
  std::cout << "cost: " << cost << std::endl;
  std::cout << "mat_H: " << std::endl
            << mat_H << std::endl
            << std::endl;
  std::cout << "covariance: " << std::endl
            << mat_H.inverse() << std::endl
            << std::endl;

  std::cout << "GT      m: " << 0.3 << " c: " << 0.1 << "\n";
  std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
  std::cout << "Final   m: " << param[0] << " c: " << param[1] << "\n";
  return 0;
}




