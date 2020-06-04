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
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <iostream>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <iomanip>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include "save_statistics.hpp"
#include "common/common.hpp"
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/utility.h"
#include "utility/visualization.h"
#include "utility/cloud_visualizer.h"

using namespace std;

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");

Estimator estimator;

SaveStatistics save_statistics;

// message buffer
queue<sensor_msgs::PointCloud2ConstPtr> cloud0_buf;
queue<sensor_msgs::PointCloud2ConstPtr> cloud1_buf;
std::mutex m_buf;

// laser path groundtruth
nav_msgs::Path laser_gt_path;
ros::Publisher pub_laser_gt_path;
Pose pose_world_ref_ini;

void saveGroundTruth()
{
    if (laser_gt_path.poses.size() == 0) return;
    std::ofstream fout(MLOAM_GT_PATH.c_str(), std::ios::out);
    fout.setf(ios::fixed, ios::floatfield);
    for (size_t i = 0; i < laser_gt_path.poses.size(); i++)
    {
        geometry_msgs::PoseStamped &laser_pose = laser_gt_path.poses[i];
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
}

void saveStatistics()
{
    printf("Saving odometry time statistics\n");
    std::ofstream fout(std::string(OUTPUT_FOLDER + "time_odometry.txt").c_str(), std::ios::out);
    fout.precision(15);
    fout << "frame, total_mea_pre_time, total_opt_odom_time" << std::endl;
    fout << estimator.frame_cnt_ << ", " << estimator.total_measurement_pre_time_ << ", " << estimator.total_opt_odom_time_ << std::endl;
    fout.close();
    ROS_WARN("Frame: %d, mean measurement preprocess time: %f, mean optimize odometry time: %f\n", estimator.frame_cnt_, 
        estimator.total_measurement_pre_time_ / estimator.frame_cnt_, estimator.total_opt_odom_time_ / estimator.frame_cnt_);        
}

void cloud0_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    m_buf.lock();
    cloud0_buf.push(cloud_msg);
    m_buf.unlock();
}

void cloud1_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    m_buf.lock();
    cloud1_buf.push(cloud_msg);
    m_buf.unlock();
}

pcl::PointCloud<pcl::PointXYZ> getCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> laser_cloud;
    pcl::fromROSMsg(*cloud_msg, laser_cloud);
    roiCloudFilter(laser_cloud, ROI_RANGE);
    return laser_cloud;
}

// extract images with same timestamp from two topics
// independent from ros::spin()
void sync_process()
{
    while(1)
    {
        if(NUM_OF_LASER == 2)
        {
            pcl::PointCloud<pcl::PointXYZ> laser_cloud0, laser_cloud1;
            std::vector<pcl::PointCloud<pcl::PointXYZ> > v_laser_cloud;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!cloud0_buf.empty() && !cloud1_buf.empty())
            {
                double time0 = cloud0_buf.front()->header.stamp.toSec();
                double time1 = cloud1_buf.front()->header.stamp.toSec();
                printf("\ntimestamps: (%.3f, %.3f)\n", time0, time1);
                // 0.07s sync tolerance
                if(time0 < time1 - LASER_SYNC_THRESHOLD)
                {
                    cloud0_buf.pop();
                    printf("throw cloud0\n");
                }
                else if(time0 > time1 + LASER_SYNC_THRESHOLD)
                {
                    cloud1_buf.pop();
                    printf("throw cloud1\n");
                }
                else
                {
                    time = cloud0_buf.front()->header.stamp.toSec();
                    header = cloud0_buf.front()->header;

                    laser_cloud0 = getCloudFromMsg(cloud0_buf.front());
                    v_laser_cloud.push_back(laser_cloud0);
                    cloud0_buf.pop();

                    laser_cloud1 = getCloudFromMsg(cloud1_buf.front());
                    v_laser_cloud.push_back(laser_cloud1);
                    cloud1_buf.pop();

                    // TODO: inject extrinsic perturbation on point clouds
                    // if (estimator.frame_cnt_ >= 0)
                    // {
                    //     ROS_WARN("Inject extrinsic perturbation on point clouds !");
                    //     Eigen::Quaterniond q_perturb(0.98929, 0.078924, 0.094058, 0.078924);
                    //     Eigen::Vector3d t_perturb(0.1, 0.1, 0.1);
                    //     Pose pose_perturb(q_perturb, t_perturb);
                    //     pcl::transformPointCloud(v_laser_cloud[1], v_laser_cloud[1], pose_perturb.T_.cast<float>());
                    // }
                    printf("size of finding laser_cloud0: %d, laser_cloud1: %d\n", laser_cloud0.points.size(), laser_cloud1.points.size());
                }
            }
            m_buf.unlock();
            if ((laser_cloud0.points.size() != 0) && (laser_cloud1.points.size() != 0))
            {
                estimator.inputCloud(time, v_laser_cloud);
            }
        }
        else if (NUM_OF_LASER == 1)
        {
            pcl::PointCloud<pcl::PointXYZ> laser_cloud0;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!cloud0_buf.empty())
            {
                time = cloud0_buf.front()->header.stamp.toSec();
                header = cloud0_buf.front()->header;
                laser_cloud0 = getCloudFromMsg(cloud0_buf.front());
                cloud0_buf.pop();
                printf("find laser_cloud0: %d \n", laser_cloud0.points.size());
            }
            m_buf.unlock();
            if (laser_cloud0.points.size() != 0)
            {
                estimator.inputCloud(time, laser_cloud0);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data != 0)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
}

void pose_gt_callback(const geometry_msgs::PoseStamped &pose_msg)
{
    Pose pose_world_base(pose_msg.pose);
    Pose pose_base_ref(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
    Pose pose_world_ref(pose_world_base * pose_base_ref);
    if (laser_gt_path.poses.size() == 0)
        pose_world_ref_ini = pose_world_ref;
    Pose pose_ref_ini_cur(pose_world_ref_ini.inverse() * pose_world_ref);

    nav_msgs::Odometry laser_odom;
    laser_odom.header = pose_msg.header;
    laser_odom.header.frame_id = "/world";
    laser_odom.child_frame_id = "/gt";
    laser_odom.pose.pose.orientation.x = pose_ref_ini_cur.q_.x();
    laser_odom.pose.pose.orientation.y = pose_ref_ini_cur.q_.y();
    laser_odom.pose.pose.orientation.z = pose_ref_ini_cur.q_.z();
    laser_odom.pose.pose.orientation.w = pose_ref_ini_cur.q_.w();
    laser_odom.pose.pose.position.x = pose_ref_ini_cur.t_(0);
    laser_odom.pose.pose.position.y = pose_ref_ini_cur.t_(1);
    laser_odom.pose.pose.position.z = pose_ref_ini_cur.t_(2);
    publishTF(laser_odom);

    geometry_msgs::PoseStamped laser_pose;
    laser_pose.header = pose_msg.header;
    laser_pose.header.frame_id = "/world";
    laser_pose.pose = laser_odom.pose.pose;
    laser_gt_path.header = laser_pose.header;
    laser_gt_path.poses.push_back(laser_pose);
    pub_laser_gt_path.publish(laser_gt_path);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("please intput: rosrun mloam mloam_node_rhd [config file] \n"
               "for example: "
               "rosrun mloam mloam_node_rhd "
               "~/catkin_ws/src/M-LOAM/config/config_handheld.yaml"
               "/Monster xxx \n");
        return 1;
    }
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "mloam_node_rhd");
    ros::NodeHandle nh("~");

    // ******************************************
    cout << "config_file: " << FLAGS_config_file << endl;
    readParameters(FLAGS_config_file);
    estimator.setParameter();
    registerPub(nh);

    MLOAM_RESULT_SAVE = FLAGS_result_save;
    printf("save result (0/1): %d\n", MLOAM_RESULT_SAVE);
    OUTPUT_FOLDER = FLAGS_output_path;
    MLOAM_GT_PATH = OUTPUT_FOLDER + "stamped_groundtruth.txt";
    MLOAM_ODOM_PATH = OUTPUT_FOLDER + "stamped_mloam_odom_estimate.txt";
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "extrinsic_parameter.txt";
    EX_CALIB_EIG_PATH = OUTPUT_FOLDER + "calib_eig.txt";
    ROS_WARN("waiting for cloud...");
    if (NUM_OF_LASER > 2)
    {
        printf("not support > 2 cases");
        ROS_BREAK();
        return 0;
    }

    // ******************************************
    ros::Subscriber sub_cloud0, sub_cloud1;
    if (NUM_OF_LASER == 1)
    {
        CLOUD0_TOPIC = CLOUD_TOPIC[0];
        sub_cloud0 = nh.subscribe(CLOUD0_TOPIC, 5, cloud0_callback);
    } else
    {
        CLOUD0_TOPIC = CLOUD_TOPIC[0];
        CLOUD1_TOPIC = CLOUD_TOPIC[1];
        sub_cloud0 = nh.subscribe(CLOUD0_TOPIC, 5, cloud0_callback);
        sub_cloud1 = nh.subscribe(CLOUD1_TOPIC, 5, cloud1_callback);       
    }
    
    ros::Subscriber sub_restart = nh.subscribe("/mlod_restart", 5, restart_callback);
    ros::Subscriber sub_pose_gt = nh.subscribe("/base_pose_gt", 5, pose_gt_callback);
    pub_laser_gt_path = nh.advertise<nav_msgs::Path>("/laser_gt_path", 5);

    std::thread sync_thread(sync_process);
    std::thread cloud_visualizer_thread;
    if (PCL_VIEWER)
    {
        cloud_visualizer_thread = std::thread(&PlaneNormalVisualizer::Spin, &estimator.plane_normal_vis_);
    }
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (MLOAM_RESULT_SAVE)
    {
        std::cout << common::RED << "saving odometry results" << common::RESET << std::endl;
        save_statistics.saveSensorPath(MLOAM_GT_PATH, laser_gt_path);
        save_statistics.saveOdomStatistics(EX_CALIB_EIG_PATH, EX_CALIB_RESULT_PATH, MLOAM_ODOM_PATH, estimator);
        save_statistics.saveOdomTimeStatistics(OUTPUT_FOLDER + "time_odometry_txt", estimator);
    }

    cloud_visualizer_thread.join();
    sync_thread.join();
    return 0;
}




//
