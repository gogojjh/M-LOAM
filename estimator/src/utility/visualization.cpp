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
    for (size_t i = 0; i < NUM_OF_LASER; i++)
    {
        std::string laser_odom_topic, laser_path_topic;
        laser_odom_topic = std::string("/laser_odom_") + std::to_string(i);
        v_pub_laser_odometry.push_back(nh.advertise<nav_msgs::Odometry>(laser_odom_topic, 100));
        laser_path_topic = std::string("/laser_odom_path_") + std::to_string(i);
        v_pub_laser_path.push_back(nh.advertise<nav_msgs::Path>(laser_path_topic, 100));
    }
    v_laser_path.resize(NUM_OF_LASER);
}

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    PointICloud laser_cloud, corner_points_sharp, corner_points_less_sharp, surf_points_flat, surf_points_less_flat;
    for (size_t i = 0; i < estimator.cur_feature_.second.size(); i++)
    {
        cloudFeature cloud_feature_trans = transformCloudFeature(estimator.cur_feature_.second[i], estimator.pose_base_laser_[i].T_.cast<float>());
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
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag_ != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    // ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    // ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    // if (ESTIMATE_EXTRINSIC)
    // {
    //     cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
    //     for (int i = 0; i < NUM_OF_CAM; i++)
    //     {
    //         //ROS_DEBUG("calibration result for camera %d", i);
    //         ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
    //         ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
    //
    //         Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
    //         eigen_T.block<3, 3>(0, 0) = estimator.ric[i];
    //         eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
    //         cv::Mat cv_T;
    //         cv::eigen2cv(eigen_T, cv_T);
    //         if(i == 0)
    //             fs << "body_T_cam0" << cv_T ;
    //         else
    //             fs << "body_T_cam1" << cv_T ;
    //     }
    //     fs.release();
    // }
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

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag_ == Estimator::SolverFlag::INITIAL)
    {
        for (size_t i = 0; i < NUM_OF_LASER; i++)
        {
            nav_msgs::Odometry ext_base_to_laser;
            ext_base_to_laser.header = header;
            ext_base_to_laser.header.frame_id = "/world";
            ext_base_to_laser.child_frame_id = "/laser_init_" + std::to_string(i);
            Pose pose_base_laser = estimator.pose_base_laser_[i];
            ext_base_to_laser.pose.pose.orientation.x = pose_base_laser.q_.x();
            ext_base_to_laser.pose.pose.orientation.y = pose_base_laser.q_.y();
            ext_base_to_laser.pose.pose.orientation.z = pose_base_laser.q_.z();
            ext_base_to_laser.pose.pose.orientation.w = pose_base_laser.q_.w();
            ext_base_to_laser.pose.pose.position.x = pose_base_laser.t_.x();
            ext_base_to_laser.pose.pose.position.y = pose_base_laser.t_.y();
            ext_base_to_laser.pose.pose.position.z = pose_base_laser.t_.z();
            pub_ext_base_to_sensor.publish(ext_base_to_laser);
            publishTF(ext_base_to_laser);

            nav_msgs::Odometry laser_odometry;
            laser_odometry.header = header;
            // laser_odometry.header.frame_id = "world";
            // laser_odometry.child_frame_id = "/laser_" + std::to_string(i);
            // Pose pose_w_curr = Pose::poseTransform(estimator.pose_base_laser_[i], estimator.pose_laser_cur_[i]);
            laser_odometry.header.frame_id = "/laser_init_" + std::to_string(i);
            laser_odometry.child_frame_id = "/laser_" + std::to_string(i);
            Pose pose_laser_curr = estimator.pose_laser_cur_[i];
            laser_odometry.pose.pose.orientation.x = pose_laser_curr.q_.x();
            laser_odometry.pose.pose.orientation.y = pose_laser_curr.q_.y();
            laser_odometry.pose.pose.orientation.z = pose_laser_curr.q_.z();
            laser_odometry.pose.pose.orientation.w = pose_laser_curr.q_.w();
            laser_odometry.pose.pose.position.x = pose_laser_curr.t_.x();
            laser_odometry.pose.pose.position.y = pose_laser_curr.t_.y();
            laser_odometry.pose.pose.position.z = pose_laser_curr.t_.z();
            v_pub_laser_odometry[i].publish(laser_odometry);
            publishTF(laser_odometry);

            geometry_msgs::PoseStamped laser_pose;
            laser_pose.header = laser_odometry.header;
            laser_pose.pose = laser_odometry.pose.pose;
            v_laser_path[i].header = laser_odometry.header;
            v_laser_path[i].poses.push_back(laser_pose);
            v_pub_laser_path[i].publish(v_laser_path[i]);
        }
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
