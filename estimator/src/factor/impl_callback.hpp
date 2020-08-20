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

#pragma once
#include <ceres/ceres.h>

namespace ceres
{
    class StateUpdatingCovarianceCallback : public IterationCallback
    {
    public:
        StateUpdatingCovarianceCallback(Problem *problem, double *param);
        virtual ~StateUpdatingCovarianceCallback();
        virtual CallbackReturnType operator()(const IterationSummary &summary);

        double covariance_pose_[7 * 7];

    private:
        double *param_;
        Problem *problem_;
    };
};    // namespace ceres
