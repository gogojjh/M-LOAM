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

#include "lidar_tracker.h"

using namespace common;

LidarTracker::LidarTracker()
{
    std::cout << "Tracker begin" << std::endl;
}

Pose LidarTracker::trackCloud(const cloudFeature &prev_cloud_feature,
                              const cloudFeature &cur_cloud_feature,
                              const Pose &pose_ini)
{
    pcl::KdTreeFLANN<PointI>::Ptr kdtree_corner_last(new pcl::KdTreeFLANN<PointI>());
    pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_last(new pcl::KdTreeFLANN<PointI>());

    // step 1: prev feature
    PointICloudPtr corner_points_last = boost::make_shared<PointICloud>(prev_cloud_feature.find("corner_points_less_sharp")->second);
    PointICloudPtr surf_points_last = boost::make_shared<PointICloud>(prev_cloud_feature.find("surf_points_less_flat")->second);
    kdtree_corner_last->setInputCloud(corner_points_last);
    kdtree_surf_last->setInputCloud(surf_points_last);

    // step 2: current feature
    PointICloudPtr corner_points_sharp = boost::make_shared<PointICloud>(cur_cloud_feature.find("corner_points_sharp")->second);
    PointICloudPtr surf_points_flat = boost::make_shared<PointICloud>(cur_cloud_feature.find("surf_points_flat")->second);

    // step 3: set initial pose
    double para_pose[SIZE_POSE] = {pose_ini.t_(0), pose_ini.t_(1), pose_ini.t_(2),
                                   pose_ini.q_.x(), pose_ini.q_.y(), pose_ini.q_.z(), pose_ini.q_.w()};

    for (size_t iter_cnt = 0; iter_cnt < 2; iter_cnt++)
    {
        ceres::Problem problem;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        PoseLocalParameterization *local_parameterization = new PoseLocalParameterization();      
        local_parameterization->setParameter();
        problem.AddParameterBlock(para_pose, SIZE_POSE, local_parameterization);

        // prepare feature data
        TicToc t_prepare;
        std::vector<PointPlaneFeature> corner_scan_features, surf_scan_features;
        Pose pose_local = Pose(Eigen::Quaterniond(para_pose[6], para_pose[3], para_pose[4], para_pose[5]),
                               Eigen::Vector3d(para_pose[0], para_pose[1], para_pose[2]));
        
        f_extract_.matchCornerFromScan(kdtree_corner_last, *corner_points_last, *corner_points_sharp, pose_local, corner_scan_features);
        f_extract_.matchSurfFromScan(kdtree_surf_last, *surf_points_last, *surf_points_flat, pose_local, surf_scan_features);
        
        size_t corner_num = corner_scan_features.size();
        size_t surf_num = surf_scan_features.size();
        // printf("[mloam] iter:%d, use_corner:%d, use_surf:%d\n", iter_cnt, corner_num, surf_num);
        if (corner_num + surf_num < 10)
        {
            printf("[lidar_tracker] less correspondence! *************************************************\n");
            continue;
        }

        for (const PointPlaneFeature &feature : surf_scan_features)
        {
            double s = 1.0;
            // if (DISTORTION)
            //     s = (surf_points_flat->points[idx].intensity - int(surf_points_flat->points[idx].intensity)) / SCAN_PERIOD;
            // else
            //     s = 1.0;
            LidarScanPlaneNormFactor *f = new LidarScanPlaneNormFactor(feature.point_, feature.coeffs_, s);
            problem.AddResidualBlock(f, loss_function, para_pose);
        }

        CHECK_JACOBIAN = 0;
        for (const PointPlaneFeature &feature : corner_scan_features)
        {
            double s = 1.0;
            // if (DISTORTION)
            //     s = (corner_points_sharp->points[feature.idx_].intensity - int(corner_points_sharp->points[idx].intensity)) / SCAN_PERIOD;
            // else
            //     s = 1.0;
            LidarScanEdgeFactorVector *f = new LidarScanEdgeFactorVector(feature.point_, feature.coeffs_, s);
            problem.AddResidualBlock(f, loss_function, para_pose);
            if (CHECK_JACOBIAN)
            {
                double **tmp_param = new double *[1];
                tmp_param[0] = para_pose;
                f->check(tmp_param);
                CHECK_JACOBIAN = 0;
                // delete[] tmp_param;
            }
        }

        // double cost = 0.0;
        // ceres::CRSMatrix jaco;
        // ceres::Problem::EvaluateOptions e_option;
        // e_option.parameter_blocks = para_ids;
        // e_option.residual_blocks = res_ids_proj;
        // problem.Evaluate(e_option, &cost, nullptr, nullptr, &jaco);
        // problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, &jaco);
        // evalDegenracy(local_parameterization, jaco);

        // step 3: optimization
        TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 4;
        // options.max_solver_time_in_seconds = 0.005;
        options.minimizer_progress_to_stdout = false;
        // options.check_gradients = false;
        // options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // std::cout << summary.BriefReport() << std::endl;
        // std::cout << summary.FullReport() << std::endl;
        // printf("solver time %f ms \n", t_solver.toc());
    }

    Pose pose_prev_cur(Eigen::Quaterniond(para_pose[6], para_pose[3], para_pose[4], para_pose[5]), 
                       Eigen::Vector3d(para_pose[0], para_pose[1], para_pose[2]));
    return pose_prev_cur;
}

void LidarTracker::evalDegenracy(PoseLocalParameterization *local_parameterization, const ceres::CRSMatrix &jaco)
{
    // printf("jacob: %d constraints, %d parameters\n", jaco.num_rows, jaco.num_cols); // 2000+, 6
    if (jaco.num_rows == 0) return;
    Eigen::SparseMatrix<double, Eigen::RowMajor> mat_J; // Jacobian is a diagonal matrix
    CRSMatrix2EigenMatrix(jaco, mat_J);
    Eigen::SparseMatrix<double, Eigen::RowMajor> mat_Jt = mat_J.transpose();
    Eigen::MatrixXd mat_JtJ = mat_Jt * mat_J;
    Eigen::Matrix<double, 6, 6> mat_H = mat_JtJ.block(0, 0, 6, 6);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > esolver(mat_H);
    Eigen::Matrix<double, 1, 6> mat_E = esolver.eigenvalues().real(); // 6*1
    Eigen::Matrix<double, 6, 6> mat_V_f = esolver.eigenvectors().real(); // 6*6, column is the corresponding eigenvector
    Eigen::Matrix<double, 6, 6> mat_V_p = mat_V_f;
    for (auto j = 0; j < mat_E.cols(); j++)
    {
        if (mat_E(0, j) < 10)
        {
            mat_V_p.col(j) = Eigen::Matrix<double, 6, 1>::Zero();
            local_parameterization->is_degenerate_ = true;
        } else
        {
            break;
        }
    }
    // std::cout << "[trackCloud] D factor: " << mat_E(0, 0) << ", D vector: " << mat_V_f.col(0).transpose() << std::endl;
    Eigen::Matrix<double, 6, 6> mat_P = (mat_V_f.transpose()).inverse() * mat_V_p.transpose(); // 6*6
    if (local_parameterization->is_degenerate_)
    {
        local_parameterization->V_update_ = mat_P;
        // std::cout << "param " << i << " is degenerate !" << std::endl;
        // std::cout << mat_P.transpose() << std::endl;
    }
}
