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
ros::Publisher pub_corner_points_sharp;
ros::Publisher pub_corner_points_less_sharp;
ros::Publisher pub_surf_points_flat;
ros::Publisher pub_surf_points_less_flat;

// local map
std::vector<ros::Publisher> v_pub_surf_points_pivot;
std::vector<ros::Publisher> v_pub_surf_points_local_map;

// odometry
ros::Publisher pub_ext_base_to_sensor;
std::vector<ros::Publisher> v_pub_laser_odometry;
std::vector<ros::Publisher> v_pub_laser_path;
std::vector<nav_msgs::Path> v_laser_path;


// should be deleted
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path;

ros::Publisher pub_image_track;

CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;

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

void registerPub(ros::NodeHandle &nh)
{
    pub_laser_cloud = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 100);
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

        std::string surf_points_pivot;
        surf_points_pivot = std::string("/surf_points_pivot_") + std::to_string(i);
        v_pub_surf_points_pivot.push_back(nh.advertise<sensor_msgs::PointCloud2>(surf_points_pivot, 100));

        std::string surf_points_local_map_topic;
        surf_points_local_map_topic = std::string("/surf_local_map_") + std::to_string(i);
        v_pub_surf_points_local_map.push_back(nh.advertise<sensor_msgs::PointCloud2>(surf_points_local_map_topic, 100));
    }
    v_laser_path.resize(NUM_OF_LASER);
}

void pubPointCloud(const Estimator &estimator, const double &time)
{
    std_msgs::Header header;
    header.frame_id = "laser_" + std::to_string(IDX_REF);
    header.stamp = ros::Time(time);
    PointICloud laser_cloud, corner_points_sharp, corner_points_less_sharp, surf_points_flat, surf_points_less_flat;
    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    for (size_t i = 0; i < estimator.cur_feature_.second.size() - 1; i++)
    {
        Pose pose_ext = Pose(estimator.qbl_[i], estimator.tbl_[i]);
        Pose pose_pivot(estimator.Qs_[pivot_idx], estimator.Ts_[pivot_idx]);
        Pose pose_j(estimator.Qs_[estimator.cir_buf_cnt_], estimator.Ts_[estimator.cir_buf_cnt_]);
        Eigen::Matrix4d transform_pivot_j = (pose_pivot.T_ * pose_ext.T_).inverse() * (pose_j.T_ * pose_ext.T_);
        cloudFeature cloud_feature_trans = transformCloudFeature(estimator.cur_feature_.second[i], transform_pivot_j.cast<float>());
        for (auto &p: cloud_feature_trans["laser_cloud"].points) p.intensity = i;

        laser_cloud += cloud_feature_trans["laser_cloud"];
        corner_points_sharp += cloud_feature_trans["corner_points_sharp"];
        corner_points_less_sharp += cloud_feature_trans["corner_points_less_sharp"];
        surf_points_flat += cloud_feature_trans["surf_points_flat"];
        surf_points_less_flat += cloud_feature_trans["surf_points_less_flat"];
    }
    publishCloud(pub_laser_cloud, header, laser_cloud);
    publishCloud(pub_corner_points_sharp, header, corner_points_sharp);
    publishCloud(pub_corner_points_less_sharp, header, corner_points_less_sharp);
    publishCloud(pub_surf_points_flat, header, surf_points_flat);
    publishCloud(pub_surf_points_less_flat, header, surf_points_less_flat);

    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        header.frame_id = "laser_" + std::to_string(n);
        // publishCloud(v_pub_surf_points_local_map[i], header, estimator.surf_points_local_map_filtered_[i]);
        publishCloud(v_pub_surf_points_local_map[n], header, estimator.surf_points_local_map_[n]);
        publishCloud(v_pub_surf_points_pivot[n], header, estimator.surf_points_pivot_map_[n]);
    }
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag_ != Estimator::SolverFlag::NON_LINEAR)
        return;

    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    // ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    // ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    if (ESTIMATE_EXTRINSIC)
    {
        ofstream fout(EX_CALIB_RESULT_PATH.c_str(), ios::out);
        fout.setf(ios::fixed, ios::floatfield);
        fout.precision(5);
        for (int i = 0; i < NUM_OF_LASER; i++)
        {
            fout << estimator.qbl_[i].x() << ", "
                << estimator.qbl_[i].y() << ", "
                << estimator.qbl_[i].z() << ", "
                << estimator.qbl_[i].w() << ", "
                << estimator.tbl_[i](0) << ", "
                << estimator.tbl_[i](1) << ", "
                << estimator.tbl_[i](2) << std::endl;
        }

    //     cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
    //         //ROS_DEBUG("calibration result for camera %d", i);
    //         ROS_DEBUG_STREAM("extirnsic tic: " << estimator.calib_base_laser_[i].t_.transpose());
    //         ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.calib_base_laser_[i].q_.toRotationMatrix()).transpose());
    //
    //         eigen_T.block<3, 3>(0, 0) = estimator.calib_base_laser_[i].q_;
    //         eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
    //         cv::Mat cv_T;
    //         cv::eigen2cv(eigen_T, cv_T);
    //         if(i == 0)
    //             fs << "body_T_cam0" << cv_T ;
    //         else
    //             fs << "body_T_cam1" << cv_T ;
    //     }
    //     fs.release();
        fout.close();
    }
    //
    // static double sum_of_time = 0;
    // static int sum_of_calculation = 0;
    // sum_of_time += t;
    // sum_of_calculation++;
    // ROS_DEBUG("vo solver costs: %f ms", t);
    // ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);
    //
    // sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    // last_path = estimator.Ps[WINDOW_SIZE];
    // ROS_DEBUG("sum of path %f", sum_of_path);
    // if (ESTIMATE_TD)
    //     ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const double &time)
{
    if (estimator.solver_flag_ == Estimator::SolverFlag::INITIAL)
    {
        for (size_t i = 0; i < NUM_OF_LASER; i++)
        {
            nav_msgs::Odometry ext_base_to_sensor;
            ext_base_to_sensor.header.stamp = ros::Time(time);
            ext_base_to_sensor.header.frame_id = "/world";
            ext_base_to_sensor.child_frame_id = "/laser_init_" + std::to_string(i);
            ext_base_to_sensor.pose.pose.orientation.x = estimator.qbl_[i].x();
            ext_base_to_sensor.pose.pose.orientation.y = estimator.qbl_[i].y();
            ext_base_to_sensor.pose.pose.orientation.z = estimator.qbl_[i].z();
            ext_base_to_sensor.pose.pose.orientation.w = estimator.qbl_[i].w();
            ext_base_to_sensor.pose.pose.position.x = estimator.tbl_[i](0);
            ext_base_to_sensor.pose.pose.position.y = estimator.tbl_[i](1);
            ext_base_to_sensor.pose.pose.position.z = estimator.tbl_[i](2);
            pub_ext_base_to_sensor.publish(ext_base_to_sensor);
            publishTF(ext_base_to_sensor);

            nav_msgs::Odometry laser_odometry;
            laser_odometry.header.stamp = ros::Time(time);
            laser_odometry.header.frame_id = "/laser_init_" + std::to_string(i);
            laser_odometry.child_frame_id = "/laser_" + std::to_string(i);
            Pose pose_laser_cur = estimator.pose_laser_cur_[i];
            laser_odometry.pose.pose.orientation.x = pose_laser_cur.q_.x();
            laser_odometry.pose.pose.orientation.y = pose_laser_cur.q_.y();
            laser_odometry.pose.pose.orientation.z = pose_laser_cur.q_.z();
            laser_odometry.pose.pose.orientation.w = pose_laser_cur.q_.w();
            laser_odometry.pose.pose.position.x = pose_laser_cur.t_.x();
            laser_odometry.pose.pose.position.y = pose_laser_cur.t_.y();
            laser_odometry.pose.pose.position.z = pose_laser_cur.t_.z();
            v_pub_laser_odometry[i].publish(laser_odometry);
            publishTF(laser_odometry);

            geometry_msgs::PoseStamped laser_pose;
            laser_pose.header = laser_odometry.header;
            laser_pose.pose = laser_odometry.pose.pose;
            v_laser_path[i].header = laser_odometry.header;
            v_laser_path[i].poses.push_back(laser_pose);
            v_pub_laser_path[i].publish(v_laser_path[i]);
        }
    } else
    if (estimator.solver_flag_ == Estimator::SolverFlag::NON_LINEAR)
    {
        for (size_t i = 0; i < NUM_OF_LASER; i++)
        {
            nav_msgs::Odometry ext_base_to_sensor;
            ext_base_to_sensor.header.stamp = ros::Time(time);
            ext_base_to_sensor.header.frame_id = "/world";
            ext_base_to_sensor.child_frame_id = "/laser_init_" + std::to_string(i);
            ext_base_to_sensor.pose.pose.orientation.x = estimator.qbl_[i].x();
            ext_base_to_sensor.pose.pose.orientation.y = estimator.qbl_[i].y();
            ext_base_to_sensor.pose.pose.orientation.z = estimator.qbl_[i].z();
            ext_base_to_sensor.pose.pose.orientation.w = estimator.qbl_[i].w();
            ext_base_to_sensor.pose.pose.position.x = estimator.tbl_[i](0);
            ext_base_to_sensor.pose.pose.position.y = estimator.tbl_[i](1);
            ext_base_to_sensor.pose.pose.position.z = estimator.tbl_[i](2);
            pub_ext_base_to_sensor.publish(ext_base_to_sensor);
            publishTF(ext_base_to_sensor);
            if (i != IDX_REF)
            {
                ext_base_to_sensor.header.stamp = ros::Time(time);
                ext_base_to_sensor.header.frame_id = "/laser_" + std::to_string(IDX_REF);
                ext_base_to_sensor.child_frame_id = "/laser_" + std::to_string(i);
                publishTF(ext_base_to_sensor);
            }
        }
        nav_msgs::Odometry laser_odometry;
        laser_odometry.header.stamp = ros::Time(time);
        laser_odometry.header.frame_id = "/laser_init_0";
        laser_odometry.child_frame_id = "/laser_0";
        int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
        Pose pose_laser_cur(estimator.Qs_[pivot_idx], estimator.Ts_[pivot_idx]);
        laser_odometry.pose.pose.orientation.x = pose_laser_cur.q_.x();
        laser_odometry.pose.pose.orientation.y = pose_laser_cur.q_.y();
        laser_odometry.pose.pose.orientation.z = pose_laser_cur.q_.z();
        laser_odometry.pose.pose.orientation.w = pose_laser_cur.q_.w();
        laser_odometry.pose.pose.position.x = pose_laser_cur.t_.x();
        laser_odometry.pose.pose.position.y = pose_laser_cur.t_.y();
        laser_odometry.pose.pose.position.z = pose_laser_cur.t_.z();
        v_pub_laser_odometry[0].publish(laser_odometry);
        publishTF(laser_odometry);

        geometry_msgs::PoseStamped laser_pose;
        laser_pose.header = laser_odometry.header;
        laser_pose.pose = laser_odometry.pose.pose;
        v_laser_path[0].header = laser_odometry.header;
        v_laser_path[0].poses.push_back(laser_pose);
        v_pub_laser_path[0].publish(v_laser_path[0]);
    }

    // if (estimator.solver_flag_ == Estimator::SolverFlag::NON_LINEAR)
    // {
    //     nav_msgs::Odometry odometry;
    //     odometry.header = header;
    //     odometry.header.frame_id = "world";
    //     odometry.child_frame_id = "world";
    //     Quaterniond tmp_Q;
    //     tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
    //     odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
    //     odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
    //     odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
    //     odometry.pose.pose.orientation.x = tmp_Q.x();
    //     odometry.pose.pose.orientation.y = tmp_Q.y();
    //     odometry.pose.pose.orientation.z = tmp_Q.z();
    //     odometry.pose.pose.orientation.w = tmp_Q.w();
    //     odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    //     odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
    //     odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
    //     pub_odometry.publish(odometry);
    //
    //     geometry_msgs::PoseStamped pose_stamped;
    //     pose_stamped.header = header;
    //     pose_stamped.header.frame_id = "world";
    //     pose_stamped.pose = odometry.pose.pose;
    //     path.header = header;
    //     path.header.frame_id = "world";
    //     path.poses.push_back(pose_stamped);
    //     pub_path.publish(path);
    //
    //     // write result to file
    //     ofstream foutC(VINS_RESULT_PATH, ios::app);
    //     foutC.setf(ios::fixed, ios::floatfield);
    //     foutC.precision(0);
    //     foutC << header.stamp.toSec() * 1e9 << ",";
    //     foutC.precision(5);
    //     foutC << estimator.Ps[WINDOW_SIZE].x() << ","
    //           << estimator.Ps[WINDOW_SIZE].y() << ","
    //           << estimator.Ps[WINDOW_SIZE].z() << ","
    //           << tmp_Q.w() << ","
    //           << tmp_Q.x() << ","
    //           << tmp_Q.y() << ","
    //           << tmp_Q.z() << ","
    //           << estimator.Vs[WINDOW_SIZE].x() << ","
    //           << estimator.Vs[WINDOW_SIZE].y() << ","
    //           << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
    //     foutC.close();
    //     Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
    //     printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x(), tmp_T.y(), tmp_T.z(),
    //                                                       tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
    // }
}

// void pubTF(const Estimator &estimator, const std_msgs::Header &header)
// {
//     if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
//         return;
//     static tf::TransformBroadcaster br;
//     tf::Transform transform;
//     tf::Quaternion q;
//     // body frame
//     Vector3d correct_t;
//     Quaterniond correct_q;
//     correct_t = estimator.Ps[WINDOW_SIZE];
//     correct_q = estimator.Rs[WINDOW_SIZE];
//
//     transform.setOrigin(tf::Vector3(correct_t(0),
//                                     correct_t(1),
//                                     correct_t(2)));
//     q.setW(correct_q.w());
//     q.setX(correct_q.x());
//     q.setY(correct_q.y());
//     q.setZ(correct_q.z());
//     transform.setRotation(q);
//     br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));
//
//     // camera frame
//     transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
//                                     estimator.tic[0].y(),
//                                     estimator.tic[0].z()));
//     q.setW(Quaterniond(estimator.ric[0]).w());
//     q.setX(Quaterniond(estimator.ric[0]).x());
//     q.setY(Quaterniond(estimator.ric[0]).y());
//     q.setZ(Quaterniond(estimator.ric[0]).z());
//     transform.setRotation(q);
//     br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));
//
//
//     nav_msgs::Odometry odometry;
//     odometry.header = header;
//     odometry.header.frame_id = "world";
//     odometry.pose.pose.position.x = estimator.tic[0].x();
//     odometry.pose.pose.position.y = estimator.tic[0].y();
//     odometry.pose.pose.position.z = estimator.tic[0].z();
//     Quaterniond tmp_q{estimator.ric[0]};
//     odometry.pose.pose.orientation.x = tmp_q.x();
//     odometry.pose.pose.orientation.y = tmp_q.y();
//     odometry.pose.pose.orientation.z = tmp_q.z();
//     odometry.pose.pose.orientation.w = tmp_q.w();
//     pub_ext_base_to_laser.publish(odometry);
//
// }
