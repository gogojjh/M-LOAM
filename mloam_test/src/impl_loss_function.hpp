// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2019 Google Inc. All rights reserved.
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
//
// The LossFunction interface is the way users describe how residuals
// are converted to cost terms for the overall problem cost function.
// For the exact manner in which loss functions are converted to the
// overall cost for a problem, see problem.h.
//
// For least squares problem where there are no outliers and standard
// squared loss is expected, it is not necessary to create a loss
// function; instead passing a NULL to the problem when adding
// residuals implies a standard squared loss.
//
// For least squares problems where the minimization may encounter
// input terms that contain outliers, that is, completely bogus
// measurements, it is important to use a loss function that reduces
// their associated penalty.
//
// Consider a structure from motion problem. The unknowns are 3D
// points and camera parameters, and the measurements are image
// coordinates describing the expected reprojected position for a
// point in a camera. For example, we want to model the geometry of a
// street scene with fire hydrants and cars, observed by a moving
// camera with unknown parameters, and the only 3D points we care
// about are the pointy tippy-tops of the fire hydrants. Our magic
// image processing algorithm, which is responsible for producing the
// measurements that are input to Ceres, has found and matched all
// such tippy-tops in all image frames, except that in one of the
// frame it mistook a car's headlight for a hydrant. If we didn't do
// anything special (i.e. if we used a basic quadratic loss), the
// residual for the erroneous measurement will result in extreme error
// due to the quadratic nature of squared loss. This results in the
// entire solution getting pulled away from the optimum to reduce
// the large error that would otherwise be attributed to the wrong
// measurement.
//
// Using a robust loss function, the cost for large residuals is
// reduced. In the example above, this leads to outlier terms getting
// downweighted so they do not overly influence the final solution.
//
// What cost function is best?
//
// In general, there isn't a principled way to select a robust loss
// function. The authors suggest starting with a non-robust cost, then
// only experimenting with robust loss functions if standard squared
// loss doesn't work.

// implement loss functions which are included by the original ceres
// Reference: 
// Robust Estimation and Applications in Robotics
// Graduated Non-Convexity for Robust Spatial Perception: From Non-Minimal Solvers to Global Outlier Rejection
// https://en.wikipedia.org/wiki/M-estimator

#ifndef IMPL_CERES_LOSS_FUNCTION_H_
#define IMPL_CERES_LOSS_FUNCTION_H_

#include <memory>

#include <ceres/internal/disable_warnings.h>
#include <ceres/loss_function.h>
#include <ceres/types.h>
#include <glog/logging.h>

namespace ceres 
{

// GemanMcClureLoss.
//
// t is quadratic norm
// rho(t)   = st/(s+t)
// rho(t)'  = s^2/(s+t)^2
// rho(t)'' = 2st/(s+t)^3
// 
class CERES_EXPORT GemanMcClureLoss : public LossFunction 
{
public:
    explicit GemanMcClureLoss(double a) : a_(a), b_(a * a) {}
    void Evaluate(double, double*) const override;

private:
    const double a_;
    const double b_;
};

// SurrogateGemanMcClureLoss.
// 
// t is quadratic norm
// rho(t)   = mu*s^2*t/(s^2+t)
// rho(t)'  = mu^2*s^4/(s^2+t)^2
// rho(t)'' = -2mu^2*s^4*t/(s^2+t)^3
// 
// mu controls the convexity: large mu increase the convexity
// mu -> infinity: quadratic
// mu -> 1: the original GemanMcClureLoss
class CERES_EXPORT SurrogateGemanMcClureLoss : public LossFunction 
{
public:
    explicit SurrogateGemanMcClureLoss(double a, double mu) : a_(a), b_(a * a), mu_(mu) {}
    void Evaluate(double, double*) const override;

private:
    const double a_;
    const double b_;
    const double mu_;
};


}









#endif