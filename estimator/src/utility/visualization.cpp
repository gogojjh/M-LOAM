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

#include "visualization.h"

using namespace ros;
using namespace Eigen;
using namespace common;

// pointcloud
ros::Publisher pub_laser_cloud;
ros::Publisher pub_laser_outlier;
ros::Publisher pub_corner_points_less_sharp;
ros::Publisher pub_surf_points_less_flat;
ros::Publisher pub_surf_points_target;
ros::Publisher pub_surf_points_target_localmap;

// local map
std::vector<ros::Publisher> v_pub_surf_points_local_map;
std::vector<ros::Publisher> v_pub_surf_points_cur;
std::vector<ros::Publisher> v_pub_corner_points_local_map;

// odometry
std::vector<ros::Publisher> v_pub_laser_odom;
std::vector<ros::Publisher> v_pub_laser_path;

// extrinsics
ros::Publisher pub_extrinsics;

cloudFeature transformCloudFeature(const cloudFeature &cloud_feature, const Eigen::Matrix4f &trans, const int &n)
{
    cloudFeature trans_cloud_feature;
    for (auto iter = cloud_feature.begin(); iter != cloud_feature.end(); iter ++)
    {
        PointICloud trans_cloud;
        pcl::transformPointCloud(iter->second, trans_cloud, trans);
        // for (auto &p: trans_cloud.points) p.intensity = n + (p.intensity - int(p.intensity));
        for (auto &p: trans_cloud.points) p.intensity = n;
        trans_cloud_feature.insert(pair<std::string, PointICloud>(iter->first, trans_cloud));
    }
    return trans_cloud_feature;
}

void clearPath()
{

}

void registerPub(ros::NodeHandle &nh)
{
    pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 5);
    pub_laser_outlier = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_outlier", 5);
    pub_corner_points_less_sharp = nh.advertise<sensor_msgs::PointCloud2>("/corner_points_less_sharp", 5);
    pub_surf_points_less_flat = nh.advertise<sensor_msgs::PointCloud2>("/surf_points_less_flat", 5);
    pub_extrinsics = nh.advertise<mloam_msgs::Extrinsics>("/extrinsics", 5);
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        std::string laser_odom_topic, laser_path_topic;
        laser_odom_topic = std::string("/laser_odom_") + std::to_string(i);
        v_pub_laser_odom.push_back(nh.advertise<nav_msgs::Odometry>(laser_odom_topic, 5));
        laser_path_topic = std::string("/laser_odom_path_") + std::to_string(i);
        v_pub_laser_path.push_back(nh.advertise<nav_msgs::Path>(laser_path_topic, 5));

        // v_pub_surf_points_local_map.push_back(nh.advertise<sensor_msgs::PointCloud2>(std::string("/surf_local_map_") + std::to_string(i), 5));
        // v_pub_surf_points_cur.push_back(nh.advertise<sensor_msgs::PointCloud2>(std::string("/surf_points_cur_") + std::to_string(i), 5));

        // v_pub_corner_points_local_map.push_back(nh.advertise<sensor_msgs::PointCloud2>(std::string("/corner_local_map_") + std::to_string(i), 5));
    }
    // pub_surf_points_target_localmap = nh.advertise<sensor_msgs::PointCloud2>("/surf_points_target_localmap", 5);
    // pub_surf_points_target = nh.advertise<sensor_msgs::PointCloud2>("/surf_points_target", 5);
}

void pubPointCloud(const Estimator &estimator, const double &time)
{
    std_msgs::Header header;
    header.frame_id = "laser_" + std::to_string(IDX_REF);
    header.stamp = ros::Time(time);

    PointICloud laser_cloud, laser_cloud_proj, laser_cloud_outlier;
    PointICloud corner_points_sharp, corner_points_less_sharp, surf_points_flat, surf_points_less_flat;
    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        Pose pose_ext = Pose(estimator.qbl_[n], estimator.tbl_[n]);
        cloudFeature cloud_feature_trans = transformCloudFeature(estimator.cur_feature_.second[n], pose_ext.T_.cast<float>(), n);
        laser_cloud += cloud_feature_trans["laser_cloud"];
        if ((ESTIMATE_EXTRINSIC == 0) || (n == IDX_REF))
        {
            laser_cloud_outlier += cloud_feature_trans["laser_cloud_outlier"];
            corner_points_less_sharp += cloud_feature_trans["corner_points_less_sharp"];
            surf_points_less_flat += cloud_feature_trans["surf_points_less_flat"];
        }
    }
    publishCloud(pub_laser_cloud, header, laser_cloud);
    publishCloud(pub_laser_outlier, header, laser_cloud_outlier);    
    publishCloud(pub_corner_points_less_sharp, header, corner_points_less_sharp);
    publishCloud(pub_surf_points_less_flat, header, surf_points_less_flat);

    // publish local map
    if (estimator.solver_flag_ == Estimator::SolverFlag::NON_LINEAR)
    {
        // for (size_t n = 0; n < NUM_OF_LASER; n++)
        // {
        //     // if ((ESTIMATE_EXTRINSIC !=0) && (n != IDX_REF)) continue;
        //     header.frame_id = "laser_" + std::to_string(n);
        //     PointICloud surf_local_map_trans, corner_local_map_trans;

        //     int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE - 1; // running after slideWindow()
        //     Pose pose_ext = Pose(estimator.qbl_[n], estimator.tbl_[n]);
        //     Pose pose_pivot(estimator.Qs_[pivot_idx], estimator.Ts_[pivot_idx]);
        //     Pose pose_j(estimator.Qs_[estimator.cir_buf_cnt_-1], estimator.Ts_[estimator.cir_buf_cnt_-1]);
        //     Pose pose_j_pivot = Pose((pose_j.T_ * pose_ext.T_).inverse() * pose_pivot.T_);

        //     pcl::transformPointCloud(estimator.surf_points_local_map_filtered_[n], surf_local_map_trans, pose_j_pivot.T_.cast<float>());
        //     publishCloud(v_pub_surf_points_local_map[n], header, surf_local_map_trans);
        //     pcl::transformPointCloud(estimator.corner_points_local_map_filtered_[n], corner_local_map_trans, pose_j_pivot.T_.cast<float>());
        //     publishCloud(v_pub_corner_points_local_map[n], header, corner_local_map_trans);
        // }

        // publish target cloud in localmap
        // if ((ESTIMATE_EXTRINSIC == 1) && (estimator.surf_map_features_.size() != 0))
        // {
        //     header.frame_id = "laser_0";
        //     publishCloud(pub_surf_points_target_localmap, header, estimator.surf_points_local_map_filtered_[1]);
        //     int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
        //     PointICloud cloud_trans;
        //     for (auto &f : estimator.surf_map_features_[1][pivot_idx])
        //     {
        //         PointI p_ori;
        //         p_ori.x = f.point_.x();
        //         p_ori.y = f.point_.y();
        //         p_ori.z = f.point_.z();
        //         PointI p_sel;
        //         FeatureExtract f_extract;
        //         f_extract.pointAssociateToMap(p_ori, p_sel, estimator.pose_local_[1][pivot_idx]);
        //         cloud_trans.push_back(p_sel);
        //     }
        //     publishCloud(pub_surf_points_target, header, cloud_trans);
        // }
    }

}

void printStatistics(const Estimator &estimator, double t)
{

}

void pubOdometry(Estimator &estimator, const double &time)
{
    if (ESTIMATE_EXTRINSIC == 2)
    {
        for (auto n = 0; n < NUM_OF_LASER; n++)
        {
            nav_msgs::Odometry laser_odom;
            laser_odom.header.stamp = ros::Time(time);
            laser_odom.header.frame_id = "/world";
            laser_odom.child_frame_id = "/odom_laser_" + std::to_string(n);
            Pose pose_laser_cur = estimator.pose_laser_cur_[n];
            laser_odom.pose.pose.orientation.x = pose_laser_cur.q_.x();
            laser_odom.pose.pose.orientation.y = pose_laser_cur.q_.y();
            laser_odom.pose.pose.orientation.z = pose_laser_cur.q_.z();
            laser_odom.pose.pose.orientation.w = pose_laser_cur.q_.w();
            laser_odom.pose.pose.position.x = pose_laser_cur.t_.x();
            laser_odom.pose.pose.position.y = pose_laser_cur.t_.y();
            laser_odom.pose.pose.position.z = pose_laser_cur.t_.z();
            v_pub_laser_odom[n].publish(laser_odom);
            publishTF(laser_odom);

            geometry_msgs::PoseStamped laser_pose;
            laser_pose.header = laser_odom.header;
            laser_pose.pose = laser_odom.pose.pose;
            estimator.v_laser_path_[n].header = laser_odom.header;
            estimator.v_laser_path_[n].poses.push_back(laser_pose);
            v_pub_laser_path[n].publish(estimator.v_laser_path_[n]);
        }
    } else
    {
        Pose pose_laser_cur;
        if (estimator.solver_flag_ == Estimator::SolverFlag::INITIAL)
        {
            pose_laser_cur = estimator.pose_laser_cur_[IDX_REF];
        } else
        {
            pose_laser_cur = Pose(estimator.Qs_[estimator.cir_buf_cnt_ - 1], estimator.Ts_[estimator.cir_buf_cnt_ - 1]);
        }
        nav_msgs::Odometry laser_odom;
        laser_odom.header.stamp = ros::Time(time);
        laser_odom.header.frame_id = "/world";
        laser_odom.child_frame_id = "/odom";
        laser_odom.pose.pose.orientation.x = pose_laser_cur.q_.x();
        laser_odom.pose.pose.orientation.y = pose_laser_cur.q_.y();
        laser_odom.pose.pose.orientation.z = pose_laser_cur.q_.z();
        laser_odom.pose.pose.orientation.w = pose_laser_cur.q_.w();
        laser_odom.pose.pose.position.x = pose_laser_cur.t_.x();
        laser_odom.pose.pose.position.y = pose_laser_cur.t_.y();
        laser_odom.pose.pose.position.z = pose_laser_cur.t_.z();
        v_pub_laser_odom[IDX_REF].publish(laser_odom);
        publishTF(laser_odom);

        geometry_msgs::PoseStamped laser_pose;
        laser_pose.header = laser_odom.header;
        laser_pose.pose = laser_odom.pose.pose;
        estimator.v_laser_path_[IDX_REF].header = laser_odom.header;
        estimator.v_laser_path_[IDX_REF].poses.push_back(laser_pose);
        v_pub_laser_path[IDX_REF].publish(estimator.v_laser_path_[IDX_REF]);
    }

    // publish extrinsics
    mloam_msgs::Extrinsics extrinsics;
    extrinsics.header.stamp = ros::Time(time);
    extrinsics.header.frame_id = "/laser_" + std::to_string(IDX_REF);
    extrinsics.status = ESTIMATE_EXTRINSIC;
    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        nav_msgs::Odometry exts;
        exts.header.seq = n;
        exts.header.stamp = ros::Time(time);
        exts.header.frame_id = "/laser_" + std::to_string(IDX_REF);
        exts.child_frame_id = "/laser_" + std::to_string(n);
        exts.pose.pose.orientation.x = estimator.qbl_[n].x();
        exts.pose.pose.orientation.y = estimator.qbl_[n].y();
        exts.pose.pose.orientation.z = estimator.qbl_[n].z();
        exts.pose.pose.orientation.w = estimator.qbl_[n].w();
        exts.pose.pose.position.x = estimator.tbl_[n](0);
        exts.pose.pose.position.y = estimator.tbl_[n](1);
        exts.pose.pose.position.z = estimator.tbl_[n](2);
        for (size_t i = 0; i < 6; i++)
            for (size_t j = 0; j < 6; j++) 
                exts.pose.covariance[i * 6 + j] = float(estimator.covbl_[n](i, j));
        publishTF(exts);
        extrinsics.odoms.push_back(exts);
    }
    pub_extrinsics.publish(extrinsics);
}

//
