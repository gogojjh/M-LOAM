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

// Usage: rosrun mloam_test test_generate_noisy_bag SR01.bag SR01_noisy.bag

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <boost/foreach.hpp>

// PCL
#include <pcl/point_cloud.h> /* pcl::PointCloud */
#include <pcl/point_types.h> /* pcl::PointXYZ */
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <random>

using namespace std;

template <typename T>
struct RandomGeneratorFloat
{
    std::random_device random_device;
    std::mt19937 m_random_engine;
    std::uniform_real_distribution<T> m_dist;
    std::normal_distribution<T> m_dist_normal;
    RandomGeneratorFloat() : m_random_engine(std::random_device{}()){};
    ~RandomGeneratorFloat(){};

    T geneRandUniform(T low = 0.0, T hight = 100.0)
    {
        m_dist = std::uniform_real_distribution<T>(low, hight);
        return m_dist(m_random_engine);
    }

    T geneRandNormal(T mean = 0.0, T std = 1.0)
    {
        m_dist_normal = std::normal_distribution<T>(mean, std);
        return m_dist_normal(m_random_engine);
    }

    T *geneRandUniformArray(T low = 0.0, T hight = 100.0, size_t numbers = 100)
    {
        T *res = new T[numbers];
        m_dist = std::uniform_real_distribution<T>(low, hight);
        for (size_t i = 0; i < numbers; i++)
        {
            res[i] = m_dist(m_random_engine);
        }
        return res;
    }
};

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        return 0;
    }
    ros::init(argc, argv, "test_generate_noisy_bag");
    ros::NodeHandle nh("~");

    rosbag::Bag bag_read;
    bag_read.open(argv[1], rosbag::bagmode::Read);
    std::vector<std::string> topics{"/left/velodyne_points",
                                    "/right/velodyne_points",
                                    "/base_odom_gt"};
    rosbag::View view(bag_read, rosbag::TopicQuery(topics));

    rosbag::Bag bag_write;
    bag_write.open(argv[2], rosbag::bagmode::Write);

    float s_td = std::stof(argv[3]);

    RandomGeneratorFloat<float> rgf;
    int frame_cnt = 0;
    BOOST_FOREACH (rosbag::MessageInstance const m, view)
    {
        // std::cout << m.getTopic() << std::endl;
        if (m.getTopic() == "/left/velodyne_points" || ("/" + m.getTopic() == "/left/velodyne_points"))
        {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            pcl::PointCloud<pcl::PointXYZ> laser_cloud;
            pcl::fromROSMsg(*cloud_msg, laser_cloud);
            // std::cout << "inject measurement noise to left" << std::endl;
            for (pcl::PointXYZ &point : laser_cloud)
            {
                float *n_xyz = rgf.geneRandUniformArray(0, s_td, 3);
                point.x += n_xyz[0];
                point.y += n_xyz[1];
                point.z += n_xyz[2];
                delete n_xyz;
            }
            sensor_msgs::PointCloud2 pub_cloud_msg;
            pub_cloud_msg.header = cloud_msg->header;
            pcl::toROSMsg(laser_cloud, pub_cloud_msg);
            bag_write.write("/left/velodyne_points", cloud_msg->header.stamp, pub_cloud_msg);            
            // printf("Frame: %d\n", frame_cnt);
            frame_cnt++;
        }
        if (m.getTopic() == "/right/velodyne_points" || ("/" + m.getTopic() == "/right/velodyne_points"))
        {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            pcl::PointCloud<pcl::PointXYZ> laser_cloud;
            pcl::fromROSMsg(*cloud_msg, laser_cloud);
            // std::cout << "inject measurement noise to right" << std::endl;
            for (pcl::PointXYZ &point : laser_cloud)
            {
                float *n_xyz = rgf.geneRandUniformArray(0, s_td, 3);
                point.x += n_xyz[0];
                point.y += n_xyz[1];
                point.z += n_xyz[2];
                delete n_xyz;
            }
            sensor_msgs::PointCloud2 pub_cloud_msg;
            pub_cloud_msg.header = cloud_msg->header;
            pcl::toROSMsg(laser_cloud, pub_cloud_msg);
            bag_write.write("/right/velodyne_points", cloud_msg->header.stamp, pub_cloud_msg);
        }
        if (m.getTopic() == "/base_odom_gt" || ("/" + m.getTopic() == "/base_odom_gt"))
        {
            nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
            bag_write.write("/base_odom_gt", odom_msg->header.stamp, *odom_msg);
        }
    }
    bag_read.close();
    bag_write.close();
    return 0;
}
