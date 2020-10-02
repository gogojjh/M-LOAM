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

#include <iostream>
#include <fstream>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "estimator/estimator.h"
#include "estimator/parameters.h"

class SaveStatistics
{
public:
    void saveSensorPath(const std::string &filename, const nav_msgs::Path &sensor_path);

    void saveOdomStatistics(const string &calib_eig_filename, 
                            const string &calib_result_filename, 
                            const string &odom_filename,
                            const Estimator &estimator);
    void saveOdomTimeStatistics(const string &filename, const Estimator &estimator);

    void saveMapStatistics(const string &map_filename,
                           const string &gf_deg_factor_filename,
                           const string &gf_logdet_filename,
                           const nav_msgs::Path &laser_aft_mapped_path,
                           const std::vector<double> &gf_deg_factor_list,
                           const std::vector<double> &gf_logdet_H_list);

    void saveMapTimeStatistics(const string &map_time_filename);
};

void SaveStatistics::saveSensorPath(const string &filename, const nav_msgs::Path &sensor_path)
{
    if (sensor_path.poses.size() == 0)
        return;
    std::ofstream fout(filename.c_str(), std::ios::out);
    fout.setf(ios::fixed, ios::floatfield);
    for (size_t i = 0; i < sensor_path.poses.size(); i++)
    {
        const geometry_msgs::PoseStamped &sensor_pose = sensor_path.poses[i];
        fout.precision(15);
        fout << sensor_pose.header.stamp.toSec() << " ";
        fout.precision(8);
        fout << sensor_pose.pose.position.x << " "
             << sensor_pose.pose.position.y << " "
             << sensor_pose.pose.position.z << " "
             << sensor_pose.pose.orientation.x << " "
             << sensor_pose.pose.orientation.y << " "
             << sensor_pose.pose.orientation.z << " "
             << sensor_pose.pose.orientation.w << std::endl;
    }
}

// odom format: timestamp tx ty tz qx qy qz qw
void SaveStatistics::saveOdomStatistics(const string &calib_eig_filename, 
                                        const string &calib_result_filename, 
                                        const string &odom_filename,
                                        const Estimator &estimator)
{
    ofstream fout;

    if (estimator.log_lambda_.size() != 0)
    {
        fout.open(calib_eig_filename.c_str(), ios::out);
        fout.setf(ios::fixed, ios::floatfield);
        fout.precision(5);
        fout << "Var1" << std::endl;
        for (size_t i = 0; i < estimator.log_lambda_.size(); i++)
            fout << estimator.log_lambda_[i] << std::endl;
        fout.close();

        fout.open(calib_result_filename.c_str(), ios::out);
        for (size_t i = 0; i < estimator.log_extrinsics_.size(); i++)
        {
            fout << estimator.log_extrinsics_[i].q_.x() << ", "
                 << estimator.log_extrinsics_[i].q_.y() << ", "
                 << estimator.log_extrinsics_[i].q_.z() << ", "
                 << estimator.log_extrinsics_[i].q_.w() << ", "
                 << estimator.log_extrinsics_[i].t_(0) << ", "
                 << estimator.log_extrinsics_[i].t_(1) << ", "
                 << estimator.log_extrinsics_[i].t_(2) << std::endl;
        }
        for (size_t i = 0; i < estimator.covbl_.size(); i++)
        {
            fout << "extrinsic covariance for the " << i << " LiDAR: " << std::endl
                 << estimator.covbl_[i] << std::endl << std::endl;
        }
        fout.close();
    }

    fout.open(odom_filename.c_str(), ios::out);
    for (size_t i = 0; i < estimator.v_laser_path_[IDX_REF].poses.size(); i++)
    {
        const geometry_msgs::PoseStamped &sensor_pose = estimator.v_laser_path_[IDX_REF].poses[i];
        fout.precision(15);
        fout << sensor_pose.header.stamp.toSec() << " ";
        fout.precision(8);
        fout << sensor_pose.pose.position.x << " "
             << sensor_pose.pose.position.y << " "
             << sensor_pose.pose.position.z << " "
             << sensor_pose.pose.orientation.x << " "
             << sensor_pose.pose.orientation.y << " "
             << sensor_pose.pose.orientation.z << " "
             << sensor_pose.pose.orientation.w << std::endl;
    }
    fout.close();
}

void SaveStatistics::saveOdomTimeStatistics(const string &filename, const Estimator &estimator)
{
    std::ofstream fout(filename.c_str(), std::ios::out);
    fout.precision(15);
    fout << "frame, corner_feature, surf_feature" << std::endl;
    fout << estimator.frame_cnt_ << ", "
         << 1.0 * estimator.total_corner_feature_ / estimator.frame_cnt_ << ", "
         << 1.0 * estimator.total_surf_feature_ / estimator.frame_cnt_ << std::endl;
    fout << "frame, mea_pre_time, matching_time, solver_time, marginalization_time, opt_odom_time" << std::endl;
    fout << common::timing::Timing::GetNumSamples("odom_mea_pre") << ", " << common::timing::Timing::GetMeanSeconds("odom_mea_pre") * 1000 << ", " << common::timing::Timing::GetSTDSeconds("odom_mea_pre") * 1000 << std::endl;
    fout << common::timing::Timing::GetNumSamples("odom_match_feat") << ", " << common::timing::Timing::GetMeanSeconds("odom_match_feat") * 1000 << ", " << common::timing::Timing::GetSTDSeconds("odom_match_feat") * 1000 << std::endl;
    fout << common::timing::Timing::GetNumSamples("odom_solver") << ", " << common::timing::Timing::GetMeanSeconds("odom_solver") * 1000 << ", " << common::timing::Timing::GetSTDSeconds("odom_solver") * 1000 << std::endl;
    fout << common::timing::Timing::GetNumSamples("odom_marg") << ", " << common::timing::Timing::GetMeanSeconds("odom_marg") * 1000 << ", " << common::timing::Timing::GetSTDSeconds("odom_marg") * 1000 << std::endl;
    fout << common::timing::Timing::GetNumSamples("odom_process") << ", " << common::timing::Timing::GetMeanSeconds("odom_process") * 1000 << ", " << common::timing::Timing::GetSTDSeconds("odom_process") * 1000 << std::endl;
    fout.close();
}

void SaveStatistics::saveMapStatistics(const string &map_filename,
                                       const string &gf_deg_factor_filename,
                                       const string &gf_logdet_filename,
                                       const nav_msgs::Path &laser_aft_mapped_path,
                                       const std::vector<double> &gf_deg_factor_list,
                                       const std::vector<double> &gf_logdet_H_list)
{
    printf("Saving mapping statistics\n");
    std::ofstream fout(map_filename.c_str(), std::ios::out);
    for (size_t i = 0; i < laser_aft_mapped_path.poses.size(); i++)
    {
        const geometry_msgs::PoseStamped &laser_pose = laser_aft_mapped_path.poses[i];
        fout.precision(15);
        fout << laser_pose.header.stamp.toSec() << " ";
        fout.precision(8);
        fout << laser_pose.pose.position.x << " "
             << laser_pose.pose.position.y << " "
             << laser_pose.pose.position.z << " "
             << laser_pose.pose.orientation.x << " "
             << laser_pose.pose.orientation.y << " "
             << laser_pose.pose.orientation.z << " "
             << laser_pose.pose.orientation.w << std::endl;
    }
    fout.close();

    fout.open(gf_deg_factor_filename.c_str(), std::ios::out);
    fout << "gf_deg_factor_list" << std::endl;
    fout.precision(8);
    for (size_t i = 0; i < gf_deg_factor_list.size(); i++)
        fout << gf_deg_factor_list[i] << std::endl;
    fout.close();

    fout.open(gf_logdet_filename.c_str(), std::ios::out);
    fout << "gf_logdet_H_list" << std::endl;
    fout.precision(8);
    for (size_t i = 0; i < gf_logdet_H_list.size(); i++)
        fout << gf_logdet_H_list[i] << std::endl;
    fout.close();
}

void SaveStatistics::saveMapTimeStatistics(const string &map_time_filename)
{
    std::ofstream fout(map_time_filename.c_str(), std::ios::out);
    fout.precision(15);
    fout << "frame, feat_matching_time, solver_time, mapping_time" << std::endl;
    fout << common::timing::Timing::GetNumSamples("mapping_match_feat") << ", " << common::timing::Timing::GetMeanSeconds("mapping_match_feat") * 1000 << ", " << common::timing::Timing::GetSTDSeconds("mapping_match_feat") * 1000 << std::endl;
    fout << common::timing::Timing::GetNumSamples("mapping_solver") << ", " << common::timing::Timing::GetMeanSeconds("mapping_solver") * 1000 << ", " << common::timing::Timing::GetSTDSeconds("mapping_solver") * 1000 << std::endl;
    fout << common::timing::Timing::GetNumSamples("mapping_process") << ", " << common::timing::Timing::GetMeanSeconds("mapping_process") * 1000 << ", " << common::timing::Timing::GetSTDSeconds("mapping_process") * 1000 << std::endl;
    fout.close();
}