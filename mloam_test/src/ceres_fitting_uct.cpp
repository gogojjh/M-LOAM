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
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

struct ExponentialResidual
{
    ExponentialResidual(double x, double y)
        : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T *const param, T *residual) const
    {
        residual[0] = y_ - exp(param[0] * x_ + param[1]);
        return true;
    }

private:
    const double x_;
    const double y_;
};

void nonoiseDataCase()
{
    std::vector<double> data;
    FILE *file;
    file = std::fopen("/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_test/data/nonoisedata.txt", "r");
    double x, y;
    char ch;
    std::string str;
    fscanf(file, "%s", &str);
    while (fscanf(file, "%lf %c %lf", &x, &ch, &y) != EOF)
    {
      data.push_back(x);
      data.push_back(y);
    }
    std::fclose(file);
    size_t kNumObservations = floor(data.size() / 2);

    double *param = new double[2];
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
    Problem problem;
    problem.AddParameterBlock(param, 2, NULL);
    for (int i = 0; i < kNumObservations; ++i)
    {
        CostFunction *cost_function =
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

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    std::cout << "nonoiseDataCase:" << std::endl;
    std::cout << "GT      m: " << 0.3 << " c: " << 0.1 << "\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   m: " << param[0] << " c: " << param[1] << "\n";
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);

    nonoiseDataCase();

    // nonoiseDataWithPriorCase();

    // noiseDataWithPriorCase();
    return 0;
}
