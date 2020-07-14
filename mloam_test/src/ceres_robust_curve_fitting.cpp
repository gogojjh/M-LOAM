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

#include "impl_loss_function.hpp"
#include "../../estimator/src/utility/tic_toc.h"

using namespace ceres;

struct ExponentialResidual 
{
    ExponentialResidual(double x, double y, double w = 1.0, double p = 0.0)
        : x_(x), y_(y), w_(sqrt(w)), p_(p) {}

    template <typename T> bool operator()(const T* const m,
                                          const T* const c,
                                          T* residual) const 
    {
        residual[0] = y_ - exp(m[0] * x_ + c[0]);
        // residual[0] *= w_;
        return true;
    }

private:
    const double x_;
    const double y_;
    const double w_;
    const double p_;
};

int main(int argc, char** argv) 
{
    google::InitGoogleLogging(argv[0]);

    std::vector<double> data;
    FILE *file;
    file = std::fopen("/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_test/data/data_curve_fitting.txt", "r");
    double x, y;
    char c;
    std::string str;
    fscanf(file, "%s", &str);
    while (fscanf(file, "%lf %c %lf", &x, &c, &y) != EOF)
    {
        data.push_back(x);
        data.push_back(y);
        // std::cout << " " << x << " " << y << std::endl;
    }
    std::fclose(file);
    size_t kNumObservations = floor(data.size() / 2);
    std::cout << "data num: " << kNumObservations << std::endl;

    double m_gt = 0.3;
    double c_gt = 0.1;

    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "GT      m: " << m_gt << " c: " << c_gt << "\n";

    std::ofstream fout("/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_test/data/fitting_result.txt");
    {
        double m = 0.0;
        double c = 0.0;
        ceres::Problem problem;
    
        double max_r = 0;
        for (int i = 0; i < kNumObservations; i++)
        {
            double r = data[2 * i + 1] - exp(data[2 * i] * m + c);
            max_r = r*r > max_r ? r*r : max_r;
        }
    
        double s = 1.0;
        double mu = max_r / (s * s);
        mu = 6;
        std::cout << "mu_0: " << mu << std::endl;

        ceres::LossFunctionWrapper *loss_function = 
            new ceres::LossFunctionWrapper(new ceres::SurrogateGemanMcClureLoss(s, mu), ceres::TAKE_OWNERSHIP);
    
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        // options.max_solver_time_in_seconds = 0.015;
        // options.minimizer_progress_to_stdout = false;
        // options.check_gradients = true;
        // options.gradient_check_relative_precision = 1e-4;
    
        for (int i = 0; i < kNumObservations; ++i) 
        {
            double r = data[2 * i + 1] - exp(data[2 * i] * m + c);
            double w = pow(mu * s * s / (r * r + mu * s * s), 2.0);
            double phi = mu * s * s * pow(sqrt(w) - 1, 2.0);
            ceres::CostFunction* cost_function = 
                new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                    new ExponentialResidual(data[2 * i], data[2 * i + 1], w, phi));
            problem.AddResidualBlock(cost_function, loss_function, &m, &c);
        }    
        
        ceres::Solver::Summary summary;
        size_t cnt = 0;
        TicToc t_solver;
        while (true)
        {
            if (mu < 1) break;
            options.max_num_iterations = 5;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << "\n";
            std::cout << cnt << "th GMC-GNC Loss " << mu << ", m: " << m << " c: " << c << "\n";        
            fout << m << "," << c << ",GMC-GNC_Loss" << std::endl;
            mu /= 1.4;
            loss_function->Reset(new ceres::SurrogateGemanMcClureLoss(s, mu), ceres::TAKE_OWNERSHIP);
            cnt++;
        }    
        printf("solver: %fms\n", t_solver.toc());
    }

    {
        double m = 0.0;
        double c = 0.0;
        double mu = 1;
        double s = 10;
        ceres::Problem problem;
        ceres::LossFunctionWrapper *loss_function = 
            new ceres::LossFunctionWrapper(new ceres::SurrogateGemanMcClureLoss(s, mu), ceres::TAKE_OWNERSHIP);
      
        for (int i = 0; i < kNumObservations; ++i) 
        {
            double r = data[2 * i + 1] - exp(data[2 * i] * m + c);
            double w = pow(mu * s * s / (r * r + mu * s * s), 2.0);
            double phi = mu * s * s * pow(sqrt(w) - 1, 2.0);
            // increase r, decrease w, increase phi
            // std::cout << "r: " << r << " w: " << w << " phi: " << phi << std::endl; 
            ceres::CostFunction* cost_function = 
                new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                    new ExponentialResidual(data[2 * i], data[2 * i + 1], w, phi));
            problem.AddResidualBlock(cost_function, NULL, &m, &c);
        }
        
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        // options.max_solver_time_in_seconds = 0.015;
        options.minimizer_progress_to_stdout = false;
        // options.check_gradients = true;
        // options.gradient_check_relative_precision = 1e-4;
        options.max_num_iterations = 20;
    
        ceres::Solver::Summary summary;
        TicToc t_solver;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";
        std::cout << "BR_duality GMC Loss m: " << m << " c: " << c << "\n";        
        fout << m << "," << c << ",BR_duality_GMC_Loss" << std::endl;
        printf("solver: %fms\n", t_solver.toc());
    }

    {
        double m = 0.0;
        double c = 0.0;
        ceres::Problem problem;
        ceres::LossFunctionWrapper *loss_function = 
            new ceres::LossFunctionWrapper(new ceres::SurrogateGemanMcClureLoss(10, 1), ceres::TAKE_OWNERSHIP);
      
        for (int i = 0; i < kNumObservations; ++i) 
        {
            ceres::CostFunction* cost_function = 
                new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                    new ExponentialResidual(data[2 * i], data[2 * i + 1]));
            problem.AddResidualBlock(cost_function, loss_function, &m, &c);
        }
        
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        // options.max_solver_time_in_seconds = 0.015;
        options.minimizer_progress_to_stdout = false;
        // options.check_gradients = true;
        // options.gradient_check_relative_precision = 1e-4;
        options.max_num_iterations = 20;
    
        ceres::Solver::Summary summary;
        TicToc t_solver;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";
        std::cout << "Normal GMC Loss m: " << m << " c: " << c << "\n";        
        fout << m << "," << c << ",Normal_GMC_Loss" << std::endl;
        printf("solver: %fms\n", t_solver.toc());
    }

    {
        double m = 0.0;
        double c = 0.0;
        ceres::Problem problem;
        ceres::LossFunctionWrapper *loss_function = 
            new ceres::LossFunctionWrapper(new ceres::HuberLoss(1), ceres::TAKE_OWNERSHIP);
      
        for (int i = 0; i < kNumObservations; ++i) 
        {
            ceres::CostFunction* cost_function = 
                new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                    new ExponentialResidual(data[2 * i], data[2 * i + 1]));
            problem.AddResidualBlock(cost_function, loss_function, &m, &c);
        }
        
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        // options.max_solver_time_in_seconds = 0.015;
        options.minimizer_progress_to_stdout = false;
        // options.check_gradients = true;
        // options.gradient_check_relative_precision = 1e-4;
        options.max_num_iterations = 20;
    
        ceres::Solver::Summary summary;
        TicToc t_solver;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";
        std::cout << "Huber Loss m: " << m << " c: " << c << "\n";        
        fout << m << "," << c << ",Huber" << std::endl;
        printf("solver: %fms\n", t_solver.toc());
    }

    {
        double m = 0.0;
        double c = 0.0;
        ceres::Problem problem;
      
        for (int i = 0; i < kNumObservations; ++i) 
        {
            ceres::CostFunction* cost_function = 
                new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                    new ExponentialResidual(data[2 * i], data[2 * i + 1]));
            problem.AddResidualBlock(cost_function, NULL, &m, &c);
        }
        
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;     
        // options.max_num_iterations = 15;

        ceres::Solver::Summary summary;
        TicToc t_solver;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";
        std::cout << "NULL Loss m: " << m << " c: " << c << "\n";        
        fout << m << "," << c << ",Null" << std::endl;
        printf("solver: %fms\n", t_solver.toc());
    }    

    fout.close();
  
    return 0;
}




