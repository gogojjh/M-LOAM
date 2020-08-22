// rosrun mloam_loop test_icp baseline_data/

#include <iostream>
#include <string>

#include <ceres/ceres.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "mloam_loop/utility/tic_toc.h"
#include "mloam_loop/utility/pose.h"
#include "mloam_loop/utility/feature_extract.hpp"
#include "mloam_loop/factor/pose_local_parameterization.h"
#include "mloam_loop/factor/lidar_map_plane_norm_factor.hpp"
#include "mloam_loop/factor/impl_loss_function.hpp"

FeatureExtract f_extract;

void WriteTrans(const std::string filepath, Eigen::Matrix4f transtemp)
{
    FILE *fid = fopen(filepath.c_str(), "w");
    fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(0, 0), transtemp(0, 1), transtemp(0, 2), transtemp(0, 3));
    fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(1, 0), transtemp(1, 1), transtemp(1, 2), transtemp(1, 3));
    fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(2, 0), transtemp(2, 1), transtemp(2, 2), transtemp(2, 3));
    fprintf(fid, "%.10f %.10f %.10f %.10f\n", 0.0f, 0.0f, 0.0f, 1.0f);
    fclose(fid);
}

void WriteCost(const char *filepath, double final_cost, double final_cost_normalize)
{
    FILE *fid = fopen(filepath, "w");
    fprintf(fid, "%f %f\n", final_cost, final_cost_normalize);
    printf("final cost, final cost normalize: %f %f\n", final_cost, final_cost_normalize);
    fclose(fid);
}

void WriteTime(const char *filepath, double time)
{
    FILE *fid = fopen(filepath, "w");
    fprintf(fid, "%.10f\n", time);
    printf("time: %.10fms\n", time);
    fclose(fid);
}

int main(int argc, char *argv[])
{
    pcl::PCDReader pcd_reader;
    pcl::PCDWriter pcd_writer;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_map(new pcl::PointCloud<pcl::PointXYZI>());
    pcd_reader.read(std::string(argv[1]) + "model.pcd", *laser_map);
    pcd_reader.read(std::string(argv[1]) + "data.pcd", *laser_cloud);

    size_t laser_map_num = laser_map->size();
    kdtree->setInputCloud(laser_map);

    double time = 0.0;
    TicToc t_optimization;
    Pose pose_relative(Eigen::Quaterniond(std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5])),
                       Eigen::Vector3d(std::stof(argv[6]), std::stof(argv[7]), std::stof(argv[8])));
    double gmc_s = 1.0;
    double gmc_mu = 20.0;
    double final_cost;
    double final_cost_normalize;
    std::vector<PointPlaneFeature> all_surf_features_old;
    for (int iter_cnt = 0; iter_cnt < std::stoi(argv[9]); iter_cnt++)
    {
        double para_pose[7];
        ceres::Problem problem;

        // if (gmc_mu < 1) break;
        ceres::LossFunctionWrapper *loss_function;
        loss_function = new ceres::LossFunctionWrapper(new ceres::HuberLoss(0.5), ceres::TAKE_OWNERSHIP);
        // loss_function = new ceres::LossFunctionWrapper(new ceres::SurrogateGemanMcClureLoss(gmc_s, gmc_mu), ceres::TAKE_OWNERSHIP);

        para_pose[0] = pose_relative.t_(0);
        para_pose[1] = pose_relative.t_(1);
        para_pose[2] = pose_relative.t_(2);
        para_pose[3] = pose_relative.q_.x();
        para_pose[4] = pose_relative.q_.y();
        para_pose[5] = pose_relative.q_.z();
        para_pose[6] = pose_relative.q_.w();

        PoseLocalParameterization *local_parameterization = new PoseLocalParameterization();
        local_parameterization->setParameter();
        problem.AddParameterBlock(para_pose, 7, local_parameterization);        

        std::vector<PointPlaneFeature> all_surf_features;
        size_t surf_num = 0;
        TicToc t_match_features;
        int n_neigh = 5;
        f_extract.matchSurfFromMap(kdtree,
                                   *laser_map,
                                   *laser_cloud,
                                   pose_relative.T_.cast<float>(),
                                   all_surf_features,
                                   n_neigh);
        surf_num = all_surf_features.size();
        for (const PointPlaneFeature &feature : all_surf_features)
        {
            Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity();
            LidarMapPlaneNormFactor *f = new LidarMapPlaneNormFactor(feature.point_, feature.coeffs_, cov_matrix);
            // problem.AddResidualBlock(f, loss_function, para_pose);
            problem.AddResidualBlock(f, NULL, para_pose);
        }

        TicToc t_solver;
        ceres::Solver::Summary summary;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        options.max_num_iterations = 10;
        // options.max_solver_time_in_seconds = 0.03;
        ceres::Solve(options, &problem, &summary);
        // if (iter_cnt == 9) std::cout << summary.BriefReport() << std::endl;
        std::cout << summary.BriefReport() << std::endl;
        // printf("solver time: %fms\n", t_solver.toc());

        final_cost = summary.final_cost;
        final_cost_normalize = summary.final_cost / problem.NumResidualBlocks();

        gmc_mu /= 1.4;

        pose_relative.q_ = Eigen::Quaterniond(para_pose[6], para_pose[3], para_pose[4], para_pose[5]);
        pose_relative.t_ = Eigen::Vector3d(para_pose[0], para_pose[1], para_pose[2]);
        pose_relative.update();
    }
    printf("ICP: %fms\n", t_optimization.toc());
    time += t_optimization.toc();
    std::cout << pose_relative.T_ << std::endl << std::endl;
    std::cout << pose_relative << std::endl << std::endl;

    pcl::transformPointCloud(*laser_cloud, *laser_cloud, pose_relative.T_.cast<float>());
    pcd_writer.write(std::string(argv[1]) + "data_icp.pcd", *laser_cloud);
    WriteTrans(std::string(argv[1]) + "output_icp.txt", pose_relative.T_.cast<float>());
    WriteCost(std::string(std::string(argv[1]) + "cost_icp.txt").c_str(), final_cost, final_cost_normalize);   
    WriteTime(std::string(std::string(argv[1]) + "time_icp.txt").c_str(), time);
    return 0;
}