
#include <iostream>
#include <string>

#include "mloam_loop/utility/tic_toc.h"
#include "mloam_loop/utility/pose.h"
#include "mloam_loop/utility/feature_extract.hpp"
#include "mloam_loop/factor/pose_local_parameterization.h"
#include "mloam_loop/factor/lidar_map_plane_norm_factor.hpp"

int main(int argc, char *argv[])
{
    pcl::PCDReader pcd_reader;
    pcl::PCDWriter pcd_writer;

    pcl::VoxelGrid<pcl::PointXYZI>::Ptr down_size_filter(new pcl::VoxelGrid<pcl::PointXYZI>());
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_map(new pcl::PointCloud<pcl::PointXYZI>());
    pcd_reader.read(argv[1], laser_cloud);
    pcd_reader.read(argv[2], laser_map);

    size_t laser_map_num = laser_map->size();
    kdtree->setInputCloud(laser_map);

    TicToc t_optimization;
    Pose pose_relative;
    double gmc_s = 1.0;
    double gmc_mu = 10.0;
    for (int iter_cnt = 0; iter_cnt < 20; iter_cnt++)
    {
        if (gmc_mu < 1) break;
        double para_pose[7];
        ceres::Problem problem;
        ceres::LossFunctionWrapper *loss_function = 
            new ceres::LossFunctionWrapper(new ceres::SurrogateGemanMcClureLoss(gmc_s, gmc_mu), ceres::TAKE_OWNERSHIP);

        para_pose[0] = pose_relative.t_(0);
        para_pose[1] = pose_relative.t_(1);
        para_pose[2] = pose_relative.t_(2);
        para_pose[3] = pose_relative.q_.x();
        para_pose[4] = pose_relative.q_.y();
        para_pose[5] = pose_relative.q_.z();
        para_pose[6] = pose_relative.q_.w();

        PoseLocalParameterization *local_parameterization = new PoseLocalParameterization();
        local_parameterization->setParameter();
        problem.AddParameterBlock(para_pose, SIZE_POSE, local_parameterization);        

        std::vector<PointPlaneFeature> all_surf_features;
        size_t surf_num = 0;
        TicToc t_match_features;
        int n_neigh = 5;
        f_extract_.matchSurfFromMap(kdtree,
                                   *laser_map,
                                   *laser_cloud_surf_ds_,
                                   pose_relative,
                                   all_surf_features,
                                   n_neigh);
        surf_num = all_surf_features.size();
        for (const PointPlaneFeature &feature : all_surf_features)
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
        options.max_num_iterations = 2;
        options.max_solver_time_in_seconds = 0.03;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        printf("solver time: %fms\n", t_solver.toc());

        gmc_mu /= 1.4;
        loss_function->Reset(new ceres::SurrogateGemanMcClureLoss(gmc_s, gmc_mu), ceres::TAKE_OWNERSHIP);        

        pose_relative.q_ = Eigen::Quaterniond(para_pose[6], para_pose[3], para_pose[4], para_pose[5]);
        pose_relative.t_ = Eigen::Vector3d(para_pose[0], para_pose[1], para_pose[2]);
        pose_relative.update();
    }
    printf("optimization time: %fs\n", t_optimization.toc() / 1000);

    std::cout << pose_relative.T_;
}