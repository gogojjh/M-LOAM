/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "lidar_tracker.h"
#include "../factor/lidar_factor.hpp"

using namespace common;

LidarTracker::LidarTracker()
{
    ROS_INFO("Tracker begin");
}

// TODO: may use closed-form ICP to do
Pose LidarTracker::trackCloud(const cloudFeature &prev_cloud_feature,
                              const cloudFeature &cur_cloud_feature,
                              const Pose &pose_ini)
{
    // step 1: prev feature and kdtree
    PointICloudPtr corner_points_last = boost::make_shared<PointICloud>(prev_cloud_feature.find("corner_points_less_sharp")->second);
    PointICloudPtr surf_points_last = boost::make_shared<PointICloud>(prev_cloud_feature.find("surf_points_less_flat")->second);
    pcl::KdTreeFLANN<PointI>::Ptr kdtree_corner_last(new pcl::KdTreeFLANN<PointI>());
    pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_last(new pcl::KdTreeFLANN<PointI>());
    kdtree_corner_last->setInputCloud(corner_points_last);
    kdtree_surf_last->setInputCloud(surf_points_last);

    // step 2: current feature
    PointICloudPtr corner_points_sharp = boost::make_shared<PointICloud>(cur_cloud_feature.find("corner_points_sharp")->second);
    PointICloudPtr surf_points_flat = boost::make_shared<PointICloud>(cur_cloud_feature.find("surf_points_flat")->second);

    // step 3: set initial pose
    Pose pose_last_curr(pose_ini);
    double para_pose[SIZE_POSE];

    for (size_t iter_cnt = 0; iter_cnt < 2; iter_cnt++)
    {
        ceres::Problem problem;
        ceres::Solver::Summary summary;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 10;
        // options.max_solver_time_in_seconds = 0.003;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;

        para_pose[0] = pose_last_curr.t_(0);
        para_pose[1] = pose_last_curr.t_(1);
        para_pose[2] = pose_last_curr.t_(2);
        para_pose[3] = pose_last_curr.q_.x();
        para_pose[4] = pose_last_curr.q_.y();
        para_pose[5] = pose_last_curr.q_.z();
        para_pose[6] = pose_last_curr.q_.w();

        PoseLocalParameterization *local_parameterization = new PoseLocalParameterization();
        local_parameterization->setParameter();
        problem.AddParameterBlock(para_pose, SIZE_POSE, local_parameterization);

        // prepare feature data
        TicToc t_prepare;
        std::vector<PointPlaneFeature> corner_scan_features, surf_scan_features;
        f_extract_.matchCornerFromScan(kdtree_corner_last, *corner_points_last, *corner_points_sharp, pose_last_curr, corner_scan_features);
        f_extract_.matchSurfFromScan(kdtree_surf_last, *surf_points_last, *surf_points_flat, pose_last_curr, surf_scan_features);

        int corner_num = corner_scan_features.size();
        int surf_num = surf_scan_features.size();
        // printf("iter:%d, use_corner:%d, use_surf:%d\n", iter_cnt, corner_num, surf_num);
        if ((corner_num + surf_num) < 10)
        {
            printf("less correspondence! *************************************************\n");
        }

        // CHECK_JACOBIAN = 1;
        for (auto &feature : corner_scan_features)
        {
            const size_t &idx = feature.idx_;
            const Eigen::Vector3d &p_data = feature.point_;
            const Eigen::Vector4d &coeff = feature.coeffs_;
            double s;
            if (DISTORTION)
                s = (corner_points_sharp->points[idx].intensity - int(corner_points_sharp->points[idx].intensity)) / SCAN_PERIOD;
            else
                s = 1.0;
            LidarScanPlaneNormFactor *f = new LidarScanPlaneNormFactor(p_data, coeff, s);
            problem.AddResidualBlock(f, loss_function, para_pose);
            // if (CHECK_JACOBIAN)
            // {
            //     double **tmp_param = new double *[1];
            //     tmp_param[0] = para_pose;
            //     f->check(tmp_param);
            //     CHECK_JACOBIAN = 0;
            // }
        }

        for (auto &feature : surf_scan_features)
        {
            const size_t &idx = feature.idx_;
            const Eigen::Vector3d &p_data = feature.point_;
            const Eigen::Vector4d &coeff = feature.coeffs_;
            double s;
            if (DISTORTION)
                s = (surf_points_flat->points[idx].intensity - int(surf_points_flat->points[idx].intensity)) / SCAN_PERIOD;
            else
                s = 1.0;            
            LidarScanPlaneNormFactor *f = new LidarScanPlaneNormFactor(p_data, coeff, s);
            problem.AddResidualBlock(f, loss_function, para_pose);
        }
        // printf("prepare tracker ceres time %f ms \n", t_prepare.toc());

        double cost = 0.0;
        problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
        // printf("cost: %f\n", cost);

        // step 3: optimization
        TicToc t_solver;
        ceres::Solve(options, &problem, &summary);
        // std::cout << summary.BriefReport() << std::endl;
        // std::cout << summary.FullReport() << std::endl;
        // printf("solver time %f ms \n", t_solver.toc());

        pose_last_curr.t_ = Eigen::Vector3d(para_pose[0], para_pose[1], para_pose[2]);
        pose_last_curr.q_ = Eigen::Quaterniond(para_pose[6], para_pose[3], para_pose[4], para_pose[5]);
    }
    
    return pose_last_curr;
}

// TODO: test ICP
// std::cout << "setting icp" << std::endl;
// pcl::IterativeClosestPoint<PointI, PointI> icp;
// icp.setMaxCorrespondenceDistance(0.1);
// icp.setTransformationEpsilon(1e-4);
// icp.setEuclideanFitnessEpsilon(0.01);
// icp.setMaximumIterations(10);

// std::cout << "icp input" << std::endl;
// PointICloudPtr ref_cloud(new PointICloud);
// *ref_cloud = prev_cloud_feature.find("laser_cloud")->second;
// PointICloudPtr target_cloud(new PointICloud);
// *target_cloud = cur_cloud_feature.find("laser_cloud")->second;
// icp.setInputSource(ref_cloud);
// icp.setInputTarget(target_cloud);
// std::cout << "icp compute" << std::endl;
// PointICloud final;
// icp.align(final);
// std::cout << "icp finish" << std::endl;
// std::cout << "has converged:" << icp.hasConverged() << ", score: " << icp.getFitnessScore() << std::endl;
// pose_last_curr = Pose(icp.getFinalTransformation().cast<double>());

// TODO: apply solution remapping
// ceres::Problem::EvaluateOptions e_option;
// double cost;
// ceres::CRSMatrix jaco;
// problem.Evaluate(e_option, &cost, NULL, NULL, &jaco);
// ceres::CRSMatrix &crs_matrix = jaco;
// Eigen::MatrixXd eigen_matrix = Eigen::MatrixXd::Zero(jaco.num_rows, crs_matrix.num_cols);
// for (int row = 0; row < crs_matrix.num_rows; row++)
// {
//     int start = crs_matrix.rows[row];
//     int end = crs_matrix.rows[row + 1] - 1;
//     for (int i = start; i <= end; i++)
//     {
//         int col = crs_matrix.cols[i];
//         double value = crs_matrix.values[i];
//         eigen_matrix(row, col) = value;
//     }
// }
// Eigen::MatrixXd &mat_A = eigen_matrix;
// Eigen::MatrixXd mat_At = mat_A.transpose(); // A^T
// Eigen::MatrixXd mat_AtA = mat_At * mat_A; // A^TA
//
// Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esolver(mat_AtA);
// Eigen::MatrixXd mat_E = esolver.eigenvalues().real();
// Eigen::MatrixXd mat_V = esolver.eigenvectors().real();
//
// printf("######### mat_E size: %d, %d\n", mat_E.rows(), mat_E.cols()); // 50x50
// // printf("######### degeneracy factor %f\n", mat_E(0, 0));
// for (size_t i = 0; i < mat_E.rows(); i++) printf("%f, ", mat_E(i, 0));
// printf("\n");
// }
// printf("optimization twice time %f \n", t_opt.toc());

// Pose pose_last_curr(q_prev_cur, t_prev_cur);
// std::cout << "tracker transform: " << pose_last_curr << std::endl;
// printf("whole tracker time %f ms \n", t_whole.toc());

//
