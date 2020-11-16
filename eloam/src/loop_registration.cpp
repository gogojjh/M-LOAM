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

// tutorial about LOAM: https://zhuanlan.zhihu.com/p/57351961

#include "mloam_loop/loop_registration.hpp"

void LoopRegistration::parseFPFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                 const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfh_feature,
                                 fgr::Points &points,
                                 fgr::Feature &features)
{
    for (size_t i = 0; i < cloud->size(); i++)
    {
        const pcl::PointXYZI &pt = cloud->points[i];
        Eigen::Vector3f pts(pt.x, pt.y, pt.z);
        points.push_back(pts);

        const pcl::FPFHSignature33 &feature = fpfh_feature->points[i];
        Eigen::VectorXf feat(33);
        for (size_t j = 0; j < 33; j++)
            feat(j) = feature.histogram[j];
        features.push_back(feat);
    }
}

std::pair<bool, Eigen::Matrix4d> LoopRegistration::performGlobalRegistration(pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_map,
                                                                             pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> point_cloud;
    point_cloud.push_back(laser_map);
    point_cloud.push_back(laser_cloud);

    TicToc t_fpfh;
    std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_feature;
    for (size_t i = 0; i < point_cloud.size(); i++)
    {
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointXYZI>());
        ne.setInputCloud(point_cloud[i]);
        ne.setSearchMethod(tree_normal);
        // ne.setKSearch(10);
        ne.setRadiusSearch(NORMAL_RADIUS);
        ne.compute(*normals);

        pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fest;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_fpfh(new pcl::search::KdTree<pcl::PointXYZI>());
        fest.setInputCloud(point_cloud[i]);
        fest.setInputNormals(normals);
        fest.setSearchMethod(tree_fpfh);
        fest.setRadiusSearch(FPFH_RADIUS);
        fest.compute(*object_features);
        fpfh_feature.push_back(object_features);
    }
    fgr::Points p1, p2;
    fgr::Feature f1, f2;
    parseFPFH(point_cloud[0], fpfh_feature[0], p1, f1);
    parseFPFH(point_cloud[1], fpfh_feature[1], p2, f2);
    printf("extract fpfh from %lu clouds: %fms\n", fpfh_feature.size(), t_fpfh.toc());

    TicToc t_fgr;
    fgr::CApp app(DIV_FACTOR,
                  USE_ABSOLUTE_SCALE,
                  MAX_CORR_DIST,
                  ITERATION_NUMBER,
                  TUPLE_SCALE,
                  TUPLE_MAX_CNT);
    app.LoadFeature(p1, f1);
    app.LoadFeature(p2, f2);
    app.NormalizePoints();
    app.AdvancedMatching();
    app.OptimizePairwise(true);
    printf("FGR: %fms\n", t_fgr.toc());

    double opti_cost = app.final_cost_normalize_;
    Eigen::Matrix4d T_relative = app.GetOutputTrans().cast<double>();
    std::cout << "opti_cost: " << opti_cost << ", rlt: \n" << T_relative << std::endl;

    std::pair<bool, Eigen::Matrix4d> result;
    if (opti_cost <= LOOP_GLOBAL_REGISTRATION_THRESHOLD)
    {
        result = make_pair(true, T_relative);
    }
    else
    {
        result = make_pair(false, T_relative);
    }
    return result;
}

// perform loop optimization between the model (as the base frame) to the keyframe point cloud
std::pair<bool, Eigen::Matrix4d> LoopRegistration::performLocalRegistration(const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf_from_map,
                                                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner_from_map,
                                                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf,
                                                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner,
                                                                            const Eigen::Matrix4d &T_ini)
{
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_from_map(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_corner_from_map(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_surf_from_map->setInputCloud(laser_cloud_surf_from_map);
    kdtree_corner_from_map->setInputCloud(laser_cloud_corner_from_map);
    double opti_cost = 1e7;

    Eigen::Matrix4d T_relative = T_ini;
    for (int iter_cnt = 0; iter_cnt < 2; iter_cnt++)
    {
        double para_pose[SIZE_POSE];
        ceres::Problem problem;
        ceres::LossFunctionWrapper *loss_function = new ceres::LossFunctionWrapper(new ceres::HuberLoss(1), ceres::TAKE_OWNERSHIP);
        // ceres::LossFunction *loss_function = new ceres::HuberLoss(1);

        Eigen::Quaterniond q_relative(T_relative.block<3, 3>(0, 0));
        Eigen::Vector3d t_relative = T_relative.block<3, 1>(0, 3);
        para_pose[0] = t_relative[0];
        para_pose[1] = t_relative[1];
        para_pose[2] = t_relative[2];
        para_pose[3] = q_relative.x();
        para_pose[4] = q_relative.y();
        para_pose[5] = q_relative.z();
        para_pose[6] = q_relative.w();

        PoseLocalParameterization *local_parameterization = new PoseLocalParameterization();
        local_parameterization->setParameter();
        problem.AddParameterBlock(para_pose, SIZE_POSE, local_parameterization);

        std::vector<PointPlaneFeature> all_surf_features, all_corner_features;
        size_t surf_num = 0, corner_num = 0;
        TicToc t_match_features;
        int n_neigh = 5;
        f_extract_.matchSurfFromMap(kdtree_surf_from_map,
                                    *laser_cloud_surf_from_map,
                                    *laser_cloud_surf,
                                    T_relative.cast<float>(),
                                    all_surf_features,
                                    n_neigh);
        surf_num = all_surf_features.size();
        f_extract_.matchCornerFromMap(kdtree_corner_from_map,
                                      *laser_cloud_corner_from_map,
                                      *laser_cloud_corner,
                                      T_relative.cast<float>(),
                                      all_corner_features,
                                      n_neigh);
        corner_num = all_corner_features.size();
        // printf("match surf: %lu, corner: %lu\n", surf_num, corner_num);
        // printf("matching features time: %fms\n", t_match_features.toc()); // 40ms
        if (1.0 * surf_num / laser_cloud_surf->size() <= 0.2 && 
            1.0 * corner_num / laser_cloud_corner->size() <= 0.2) // kf surf num: 14271, corner num: 2696
        {
            printf("not enough corresponding features\n");
            break;
        }

        for (const PointPlaneFeature &feature : all_surf_features)
        {
            // Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity() * 0.0025;
            Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity();
            LidarMapPlaneNormFactor *f = new LidarMapPlaneNormFactor(feature.point_, feature.coeffs_, cov_matrix);
            problem.AddResidualBlock(f, loss_function, para_pose);
        }
        for (const PointPlaneFeature &feature : all_corner_features)
        {
            // Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity() * 0.0025;
            Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity();
            LidarMapPlaneNormFactor *f = new LidarMapPlaneNormFactor(feature.point_, feature.coeffs_, cov_matrix);
            problem.AddResidualBlock(f, loss_function, para_pose);
        }

        TicToc t_solver;
        ceres::Solver::Summary summary;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        options.max_num_iterations = 5;
        options.max_solver_time_in_seconds = 0.03;
        ceres::Solve(options, &problem, &summary);
        // std::cout << summary.BriefReport() << std::endl;
        // opti_cost = std::min(summary.final_cost / problem.NumResidualBlocks(), opti_cost);
        opti_cost = std::min(summary.final_cost, opti_cost);

        q_relative = Eigen::Quaterniond(para_pose[6], para_pose[3], para_pose[4], para_pose[5]);
        t_relative = Eigen::Vector3d(para_pose[0], para_pose[1], para_pose[2]);
        T_relative.block<3, 3>(0, 0) = q_relative.toRotationMatrix();
        T_relative.block<3, 1>(0, 3) = t_relative;
    }
    std::cout << "opti_cost: " << opti_cost << "\nrlt: \n" << T_relative << std::endl;

    std::pair<bool, Eigen::Matrix4d> result;
    if (opti_cost <= LOOP_LOCAL_REGISTRATION_THRESHOLD)
    {
        result = make_pair(true, T_relative);
    }
    else
    {
        result = make_pair(false, T_relative);
    }
    return result;
}
