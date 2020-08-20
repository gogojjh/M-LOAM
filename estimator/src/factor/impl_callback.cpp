/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of M-LOAM (https://ram-lab.com/file/jjiao/m-loam).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Jianhao JIAO (jiaojh1994@gmail.com)
 *******************************************************/

#include "impl_callback.hpp"
#include <iostream>

namespace ceres
{
    StateUpdatingCovarianceCallback::StateUpdatingCovarianceCallback(Problem *problem,
                                                                        double *param)
        : problem_(problem), param_(param) {}

    StateUpdatingCovarianceCallback::~StateUpdatingCovarianceCallback() {}

    CallbackReturnType StateUpdatingCovarianceCallback::operator()(
        const IterationSummary &summary)
    {
        std::cout << "iteration: " << summary.iteration << std::endl;
        // TODO: evaluate covariance
        return SOLVER_CONTINUE;
    }
};    // namespace ceres