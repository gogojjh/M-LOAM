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
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <iomanip>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include "../../estimator/src/estimator/pose.h"
#include "g2oIO/PoseGraphIO.h"

#define LASER_SYNC_THRESHOLD 0.07

DEFINE_string(save_path, "pose_graph", "path to save the pose graph");
DEFINE_int32(num_laser, 1, "number of LiDAR for mapping");
DEFINE_int32(save_interval, 1, "save interval");

std::string path;
int num_laser;
int save_interval;

size_t frame_cnt = 0;
pcl::PCDWriter pcd_writer;

std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_buf;
std::queue<nav_msgs::OdometryConstPtr> odom_buf;
std::mutex m_buf;

pcl::PointCloud<pcl::PointXYZI> laser_cloud;
pcl::PointCloud<pcl::PointXYZI> laser_cloud_map;
Pose pose_cur_odom;

PoseGraphIO g2osaver;

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    m_buf.lock();
    cloud_buf.push(cloud_msg);
    m_buf.unlock();
}

void laserMapOdometryHandler(const nav_msgs::OdometryConstPtr &odom_msg)
{
    m_buf.lock();
    odom_buf.push(odom_msg);
    m_buf.unlock();
}

void sync_process()
{
    while (1)
    {
        std_msgs::Header header;
        double time = 0;
        m_buf.lock();
        if (!cloud_buf.empty() && !odom_buf.empty())
        {
            double time0 = cloud_buf.front()->header.stamp.toSec();
            double time1 = odom_buf.front()->header.stamp.toSec();
            printf("\ntimestamps: (%.3f, %.3f)\n", time0, time1);
            // 0.07s sync tolerance
            if (time0 < time1 - LASER_SYNC_THRESHOLD)
            {
                cloud_buf.pop();
                printf("throw cloud0\n");
            }
            else if (time0 > time1 + LASER_SYNC_THRESHOLD)
            {
                odom_buf.pop();
                printf("throw cloud1\n");
            }
            else
            {
                time = cloud_buf.front()->header.stamp.toSec();
                header = cloud_buf.front()->header;
                pcl::fromROSMsg(*cloud_buf.front(), laser_cloud);
                cloud_buf.pop();

                pose_cur_odom = Pose(*odom_buf.front());
                odom_buf.pop();
                printf("size of finding laser_cloud: %d\n", laser_cloud.points.size());
            }
        }
        m_buf.unlock();
        if (laser_cloud.points.size() != 0)
        {
            // pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZI>> down_size_filter(0.2, 0.2, 0.2);
            // pcl::PointCloud<pcl::PointXYZI> laser_cloud_filter;
            // down_size_filter.setInputCloud(*laser_cloud);
            // down_size_filter.filter(laser_cloud_filter);
            // laser_cloud_full_map += laser_cloud_filter;

            stringstream ss;
            ss << std::fixed << std::setprecision(9);
            ss << path << "pcd/" << time << ".pcd";
            pcd_writer.write(ss.str(), laser_cloud);
            std::cout << "Saving pcd to: " << ss.str() << std::endl;

            if (frame_cnt % save_interval == 0)
            {
                for (auto p : laser_cloud.points)
                    if (int(p.intensity) + 1 <= num_laser)
                        laser_cloud_map.points.push_back(p);
            }
            laser_cloud.clear();

            g2osaver.insertPose(Eigen::Isometry3d(pose_cur_odom.T_.cast<double>()));
            std::cout << pose_cur_odom << std::endl;

            frame_cnt++;
        }
    }
}

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        printf("please intput: rosrun mloam test_generate_full_map -help\n");
        return 1;
    }
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "test_generate_full_map");
    ros::NodeHandle nh("~");

    ros::Subscriber sub_laser_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_registered", 5, laserCloudFullResHandler);
    ros::Subscriber sub_laser_map_odometry = nh.subscribe<nav_msgs::Odometry>("/laser_map", 5, laserMapOdometryHandler);

    path = FLAGS_save_path;
    num_laser = FLAGS_num_laser;
    save_interval = FLAGS_save_interval;

    std::thread sync_thread(sync_process);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    stringstream ss;
    ss << path << "mloam_map_pose_" << num_laser << ".g2o";
    std::cout << "Saving g2o to: " << ss.str() << std::endl;
    g2osaver.saveGraph(ss.str());

    laser_cloud_map.width = 1;
    laser_cloud_map.height = laser_cloud_map.points.size();
    ss.str("");
    ss << path << "mloam_cloud_map_" << num_laser << ".pcd";
    pcd_writer.write(ss.str(), laser_cloud_map);
}
