/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"

using namespace ros;
using namespace Eigen;

using namespace common;

// pointcloud
ros::Publisher pub_laser_cloud;
ros::Publisher pub_laser_cloud_proj;
ros::Publisher pub_corner_points_sharp;
ros::Publisher pub_corner_points_less_sharp;
ros::Publisher pub_surf_points_flat;
ros::Publisher pub_surf_points_less_flat;
ros::Publisher pub_surf_points_target;
ros::Publisher pub_surf_points_target_localmap;

// local map
std::vector<ros::Publisher> v_pub_surf_points_local_map;
std::vector<ros::Publisher> v_pub_surf_points_cur;

std::vector<ros::Publisher> v_pub_corner_points_local_map;

// odometry
ros::Publisher pub_ext_base_to_sensor;
std::vector<ros::Publisher> v_pub_laser_odometry;
std::vector<ros::Publisher> v_pub_laser_path;
std::vector<nav_msgs::Path> v_laser_path;

cloudFeature transformCloudFeature(const cloudFeature &cloud_feature, const Eigen::Matrix4f &trans)
{
    cloudFeature trans_cloud_feature;
    for (auto iter = cloud_feature.begin(); iter != cloud_feature.end(); iter ++)
    {
        PointICloud trans_cloud;
        pcl::transformPointCloud(iter->second, trans_cloud, trans);
        trans_cloud_feature.insert(pair<std::string, PointICloud>(iter->first, trans_cloud));
    }
    return trans_cloud_feature;
}

void clearPath()
{

}

void registerPub(ros::NodeHandle &nh)
{
    pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 100);
    pub_laser_cloud_proj = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_proj", 100);
    pub_corner_points_sharp = nh.advertise<sensor_msgs::PointCloud2>("/corner_points_sharp", 100);
    pub_corner_points_less_sharp = nh.advertise<sensor_msgs::PointCloud2>("/corner_points_less_sharp", 100);
    pub_surf_points_flat = nh.advertise<sensor_msgs::PointCloud2>("/surf_points_flat", 100);
    pub_surf_points_less_flat = nh.advertise<sensor_msgs::PointCloud2>("/surf_points_less_flat", 100);
    pub_ext_base_to_sensor = nh.advertise<nav_msgs::Odometry>("/ext_base_to_sensor", 100);
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        std::string laser_odom_topic, laser_path_topic;
        laser_odom_topic = std::string("/laser_odom_") + std::to_string(i);
        v_pub_laser_odometry.push_back(nh.advertise<nav_msgs::Odometry>(laser_odom_topic, 100));
        laser_path_topic = std::string("/laser_odom_path_") + std::to_string(i);
        v_pub_laser_path.push_back(nh.advertise<nav_msgs::Path>(laser_path_topic, 100));

        v_pub_surf_points_local_map.push_back(nh.advertise<sensor_msgs::PointCloud2>(std::string("/surf_local_map_") + std::to_string(i), 100));
        v_pub_surf_points_cur.push_back(nh.advertise<sensor_msgs::PointCloud2>(std::string("/surf_points_cur_") + std::to_string(i), 100));

        v_pub_corner_points_local_map.push_back(nh.advertise<sensor_msgs::PointCloud2>(std::string("/corner_local_map_") + std::to_string(i), 100));
    }
    pub_surf_points_target_localmap = nh.advertise<sensor_msgs::PointCloud2>("/surf_points_target_localmap", 100);
    pub_surf_points_target = nh.advertise<sensor_msgs::PointCloud2>("/surf_points_target", 100);
    v_laser_path.resize(NUM_OF_LASER);
}

void pubPointCloud(const Estimator &estimator, const double &time)
{
    std_msgs::Header header;
    header.frame_id = "laser_" + std::to_string(IDX_REF);
    header.stamp = ros::Time(time);

    // publish raw points
    PointICloud laser_cloud, laser_cloud_proj, corner_points_sharp, corner_points_less_sharp, surf_points_flat, surf_points_less_flat;
    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        Pose pose_ext = Pose(estimator.qbl_[n], estimator.tbl_[n]);
        Eigen::Matrix4d transform_ext = pose_ext.T_;
        cloudFeature cloud_feature_trans = transformCloudFeature(estimator.cur_feature_.second[n], transform_ext.cast<float>());
        for (auto &p: cloud_feature_trans["laser_cloud"].points) p.intensity = n;
        laser_cloud_proj += cloud_feature_trans["laser_cloud"];
        if ((ESTIMATE_EXTRINSIC == 0) || (n == IDX_REF))
        {
            laser_cloud += cloud_feature_trans["laser_cloud"];
            // corner_points_sharp += cloud_feature_trans["corner_points_sharp"];
            // surf_points_flat += cloud_feature_trans["surf_points_flat"];
            corner_points_less_sharp += cloud_feature_trans["corner_points_less_sharp"];
            surf_points_less_flat += cloud_feature_trans["surf_points_less_flat"];
        }
    }
    publishCloud(pub_laser_cloud, header, laser_cloud);
    publishCloud(pub_laser_cloud_proj, header, laser_cloud_proj);
    // publishCloud(pub_corner_points_sharp, header, corner_points_sharp);
    // publishCloud(pub_surf_points_flat, header, surf_points_flat);
    publishCloud(pub_corner_points_less_sharp, header, corner_points_less_sharp);
    publishCloud(pub_surf_points_less_flat, header, surf_points_less_flat);

    // publish local map
    if (estimator.solver_flag_ == Estimator::SolverFlag::NON_LINEAR)
    {
        for (size_t n = 0; n < NUM_OF_LASER; n++)
        {
            // if ((ESTIMATE_EXTRINSIC !=0) && (n != IDX_REF)) continue;
            header.frame_id = "laser_" + std::to_string(n);
            PointICloud surf_local_map_trans, corner_local_map_trans;

            int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE - 1; // running after slideWindow()
            Pose pose_ext = Pose(estimator.qbl_[n], estimator.tbl_[n]);
            Pose pose_pivot(estimator.Qs_[pivot_idx], estimator.Ts_[pivot_idx]);
            Pose pose_j(estimator.Qs_[estimator.cir_buf_cnt_-1], estimator.Ts_[estimator.cir_buf_cnt_-1]);
            Pose pose_j_pivot = Pose((pose_j.T_ * pose_ext.T_).inverse() * pose_pivot.T_);

            pcl::transformPointCloud(estimator.surf_points_local_map_filtered_[n], surf_local_map_trans, pose_j_pivot.T_.cast<float>());
            publishCloud(v_pub_surf_points_local_map[n], header, surf_local_map_trans);
            pcl::transformPointCloud(estimator.corner_points_local_map_filtered_[n], corner_local_map_trans, pose_j_pivot.T_.cast<float>());
            publishCloud(v_pub_corner_points_local_map[n], header, corner_local_map_trans);
        }

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
    // if (estimator.solver_flag_ != Estimator::SolverFlag::NON_LINEAR) return;
    // timestamp tx ty tz qx qy qz qw
    if (ESTIMATE_EXTRINSIC != 2)
    {
        ofstream fout(EX_CALIB_RESULT_PATH.c_str(), ios::out);
        fout.setf(ios::fixed, ios::floatfield);
        fout.precision(5);
        for (auto i = 0; i < NUM_OF_LASER; i++)
        {
            fout << i << " "
                << estimator.tbl_[i](0) << " "
                << estimator.tbl_[i](1) << " "
                << estimator.tbl_[i](2) << " "
                << estimator.qbl_[i].x() << " "
                << estimator.qbl_[i].y() << " "
                << estimator.qbl_[i].z() << " "
                << estimator.qbl_[i].w() << std::endl;
        }
        fout.close();
    }

    {
        ofstream fout(MLOAM_ODOM_PATH.c_str(), ios::out);
        fout.setf(ios::fixed, ios::floatfield);
        fout.precision(5);
        for (auto i = 0; i < v_laser_path[IDX_REF].poses.size(); i++)
        {
            geometry_msgs::PoseStamped &laser_pose = v_laser_path[IDX_REF].poses[i];
            fout << laser_pose.header.stamp.toSec() << " "
                << laser_pose.pose.position.x << " "
                << laser_pose.pose.position.y << " "
                << laser_pose.pose.position.z << " "
                << laser_pose.pose.orientation.x << " "
                << laser_pose.pose.orientation.y << " "
                << laser_pose.pose.orientation.z << " "
                << laser_pose.pose.orientation.w << std::endl;
        }
        fout.close();
    }

    {
        ofstream fout(MLOAM_GT_PATH.c_str(), ios::out);
        fout.setf(ios::fixed, ios::floatfield);
        fout.precision(5);
        for (auto i = 0; i < estimator.laser_path_gt_.poses.size(); i++)
        {
            geometry_msgs::PoseStamped laser_pose = estimator.laser_path_gt_.poses[i];
            fout << laser_pose.header.stamp.toSec() << " "
                << laser_pose.pose.position.x << " "
                << laser_pose.pose.position.y << " "
                << laser_pose.pose.position.z << " "
                << laser_pose.pose.orientation.x << " "
                << laser_pose.pose.orientation.y << " "
                << laser_pose.pose.orientation.z << " "
                << laser_pose.pose.orientation.w << std::endl;
        }
        fout.close();
    }
}

void pubOdometry(const Estimator &estimator, const double &time)
{
    if (estimator.solver_flag_ == Estimator::SolverFlag::INITIAL)
    {
        for (size_t n = 0; n < NUM_OF_LASER; n++)
        {
            if ((ESTIMATE_EXTRINSIC != 2) && (n != IDX_REF)) continue; // check if extriniscs are provided
            nav_msgs::Odometry laser_odometry;
            laser_odometry.header.stamp = ros::Time(time);
            laser_odometry.header.frame_id = "/laser_init_" + std::to_string(n);
            laser_odometry.child_frame_id = "/laser_" + std::to_string(n);
            Pose pose_laser_cur = estimator.pose_laser_cur_[n];
            laser_odometry.pose.pose.orientation.x = pose_laser_cur.q_.x();
            laser_odometry.pose.pose.orientation.y = pose_laser_cur.q_.y();
            laser_odometry.pose.pose.orientation.z = pose_laser_cur.q_.z();
            laser_odometry.pose.pose.orientation.w = pose_laser_cur.q_.w();
            laser_odometry.pose.pose.position.x = pose_laser_cur.t_.x();
            laser_odometry.pose.pose.position.y = pose_laser_cur.t_.y();
            laser_odometry.pose.pose.position.z = pose_laser_cur.t_.z();
            v_pub_laser_odometry[n].publish(laser_odometry);
            publishTF(laser_odometry);

            geometry_msgs::PoseStamped laser_pose;
            laser_pose.header = laser_odometry.header;
            laser_pose.pose = laser_odometry.pose.pose;
            v_laser_path[n].header = laser_odometry.header;
            v_laser_path[n].poses.push_back(laser_pose);
            v_pub_laser_path[n].publish(v_laser_path[n]);
        }
    }
    else if (estimator.solver_flag_ == Estimator::SolverFlag::NON_LINEAR)
    {
        Pose pose_laser_cur(estimator.Qs_[estimator.cir_buf_cnt_-1], estimator.Ts_[estimator.cir_buf_cnt_-1]);
        nav_msgs::Odometry laser_odometry;
        laser_odometry.header.stamp = ros::Time(time);
        laser_odometry.header.frame_id = "/laser_init_" + std::to_string(IDX_REF);
        laser_odometry.child_frame_id = "/laser_" + std::to_string(IDX_REF);
        laser_odometry.pose.pose.orientation.x = pose_laser_cur.q_.x();
        laser_odometry.pose.pose.orientation.y = pose_laser_cur.q_.y();
        laser_odometry.pose.pose.orientation.z = pose_laser_cur.q_.z();
        laser_odometry.pose.pose.orientation.w = pose_laser_cur.q_.w();
        laser_odometry.pose.pose.position.x = pose_laser_cur.t_.x();
        laser_odometry.pose.pose.position.y = pose_laser_cur.t_.y();
        laser_odometry.pose.pose.position.z = pose_laser_cur.t_.z();
        v_pub_laser_odometry[IDX_REF].publish(laser_odometry);
        publishTF(laser_odometry);

        geometry_msgs::PoseStamped laser_pose;
        laser_pose.header = laser_odometry.header;
        laser_pose.pose = laser_odometry.pose.pose;
        v_laser_path[IDX_REF].header = laser_odometry.header;
        v_laser_path[IDX_REF].poses.push_back(laser_pose);
        v_pub_laser_path[IDX_REF].publish(v_laser_path[IDX_REF]);
    }

    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        nav_msgs::Odometry ext_base_to_sensor;
        ext_base_to_sensor.header.stamp = ros::Time(time);
        ext_base_to_sensor.header.frame_id = "/world";
        ext_base_to_sensor.child_frame_id = "/laser_init_" + std::to_string(n);
        ext_base_to_sensor.pose.pose.orientation.x = estimator.qbl_[n].x();
        ext_base_to_sensor.pose.pose.orientation.y = estimator.qbl_[n].y();
        ext_base_to_sensor.pose.pose.orientation.z = estimator.qbl_[n].z();
        ext_base_to_sensor.pose.pose.orientation.w = estimator.qbl_[n].w();
        ext_base_to_sensor.pose.pose.position.x = estimator.tbl_[n](0);
        ext_base_to_sensor.pose.pose.position.y = estimator.tbl_[n](1);
        ext_base_to_sensor.pose.pose.position.z = estimator.tbl_[n](2);
        pub_ext_base_to_sensor.publish(ext_base_to_sensor);
        publishTF(ext_base_to_sensor);

        ext_base_to_sensor.header.frame_id = "/laser_" + std::to_string(IDX_REF);
        ext_base_to_sensor.child_frame_id = "/laser_" + std::to_string(n);
        publishTF(ext_base_to_sensor);
    }
}

//
