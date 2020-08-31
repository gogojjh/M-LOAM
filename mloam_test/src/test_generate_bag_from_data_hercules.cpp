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

// Usage: rosrun mloam_test test_generate_bag_from_data_hercules \
//  /Monster/dataset/lidar_calibration/mloam_rv_dataset/ 7500 25229 1 ../RV01.bag

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include "common/common.hpp"
#include "common/publisher.hpp"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

// PCL
#include <pcl/point_cloud.h>			/* pcl::PointCloud */
#include <pcl/point_types.h>			/* pcl::PointXYZ */
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include <iostream>

using namespace std;

typedef sensor_msgs::PointCloud2 LidarMsgType;
typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType, LidarMsgType, LidarMsgType> LidarSyncPolicy;
typedef message_filters::Subscriber<LidarMsgType> LidarSubType;

ros::Publisher cloud_fused_pub;

Eigen::Quaterniond q_world_ref_ini;
Eigen::Vector3d t_world_ref_ini;
nav_msgs::Path laser_gt_path;

bool b_pause;

string MLOAM_GT_PATH;

void pauseCallback(std_msgs::StringConstPtr msg)
{
    printf("%s\n", msg->data.c_str());
    b_pause = !b_pause;
}

int main(int argc, char** argv)
{
    if (argc < 6)
    {
        return 0;
    }
    ros::init(argc, argv, "test_generate_bag_from_data_hercules");
    ros::NodeHandle nh("~");
    string data_path = argv[1];
    size_t MLOAM_START_IDX = std::stoi(argv[2]);
    size_t MLOAM_END_IDX = std::stoi(argv[3]);
    size_t MLOAM_DELTA_IDX = std::stoi(argv[4]);
    std::string BAG_NAME = std::string(argv[5]);

    std::vector<std::string> velo_topic{"/top/rslidar_points", "/front/rslidar_points", "/left/rslidar_points", "/right/rslidar_points"};
    rosbag::Bag bag;
    bag.open((data_path + BAG_NAME).c_str(), rosbag::bagmode::Write);

    // *************************************
    // read cloud list
    FILE *file;
    double base_time = ros::Time::now().toSec();
    double cloud_time;
    vector<double> cloud_time_list;
    {
        file = std::fopen((data_path + "cloud_0/timestamps.txt").c_str(), "r");
        if (!file)
        {
            printf("cannot find file: %stimes.txt\n", data_path.c_str());
            ROS_BREAK();
            return 0;
        }
        while (fscanf(file, "%lf", &cloud_time) != EOF)
        {
            cloud_time_list.push_back(cloud_time);
        }
        std::fclose(file);    
    }  

    // *************************************
    // read data
    for (size_t i = MLOAM_START_IDX; i < std::min(MLOAM_END_IDX, cloud_time_list.size()); i+=MLOAM_DELTA_IDX)
    {	
        if (ros::ok())
        {
            double cloud_time = cloud_time_list[i];
            printf("process data: %d\n", i);
            stringstream ss;
            ss << setfill('0') << setw(6) << i;

            // load cloud
            printf("size of finding cloud\n");
            std::vector<pcl::PointCloud<pcl::PointXYZI> > laser_cloud_list(4);
            for (size_t j = 0; j < 4; j++)
            {
                stringstream ss_file;
                ss_file << "cloud_" << j << "/data/" << ss.str() << ".pcd";
                string cloud_path = data_path + ss_file.str();
                if (pcl::io::loadPCDFile<pcl::PointXYZI>(cloud_path, laser_cloud_list[j]) == -1)
                {
                    printf("Couldn't read file %s\n", cloud_path.c_str());
                    ROS_BREAK();
                    return 0;
                }
            }

            for (size_t i = 0; i < 4; i++)
            {
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(laser_cloud_list[i], cloud_msg);
                cloud_msg.header.frame_id = "velo";
                cloud_msg.header.stamp = ros::Time(cloud_time);
                bag.write(velo_topic[i], ros::Time(cloud_time), cloud_msg);
            }

            // load gps
            FILE *gps_file;
            string gps_file_path = data_path + "gps/data/" + ss.str() + ".txt";
            gps_file = std::fopen(gps_file_path.c_str() , "r");
            if(!gps_file)
            {
                // printf("cannot find file: %s\n", gps_file_path.c_str());
                // ROS_BREAK();
                // return 0;          
            } else
            {
                double lat, lon, alt;
                double posx_accuracy, posy_accuracy, posz_accuracy;
                int navstat, numsats;
                fscanf(gps_file, "%d %d ", &navstat, &numsats);
                fscanf(gps_file, "%lf %lf %lf ", &lat, &lon, &alt);
                fscanf(gps_file, "%lf %lf %lf ", &posx_accuracy, &posy_accuracy, &posz_accuracy);
                std::fclose(gps_file);

                sensor_msgs::NavSatFix gps_position;
                gps_position.header.frame_id = "gps";
                gps_position.header.stamp = ros::Time(cloud_time);
                gps_position.status.status = navstat;
                gps_position.status.service = numsats;
                gps_position.latitude = lat;
                gps_position.longitude = lon;
                gps_position.altitude = alt;
                gps_position.position_covariance[0] = posx_accuracy;
                gps_position.position_covariance[4] = posy_accuracy;
                gps_position.position_covariance[8] = posz_accuracy;
                bag.write("/novatel718d/pos", ros::Time(cloud_time), gps_position);
            }
            
            // load odom
            FILE *gt_odom_file;
            string gt_odom_file_path = data_path + "gt_odom/data/" + ss.str() + ".txt";
            gt_odom_file = std::fopen(gt_odom_file_path.c_str() , "r");
            if(!gt_odom_file)
            {
                // printf("cannot find file: %s\n", gt_odom_file_path.c_str());
                // ROS_BREAK();
                // return 0;          
            } else
            {
                double posx, posy, posz;
                double orix, oriy, oriz, oriw;
                fscanf(gt_odom_file, "%lf %lf %lf ", &posx, &posy, &posz);
                fscanf(gt_odom_file, "%lf %lf %lf %lf ", &orix, &oriy, &oriz, &oriw);
                std::fclose(gt_odom_file);

                Eigen::Vector3d t_world_base(posx, posy, posz);
                Eigen::Quaterniond q_world_base(oriw, orix, oriy, oriz);
                Eigen::Quaterniond q_base_ref(1, 0, 0, 0);
                Eigen::Vector3d t_base_ref(0, 0, 0);
                Eigen::Quaterniond q_world_ref = q_world_base * q_base_ref;
                Eigen::Vector3d t_world_ref = q_world_base * t_base_ref + t_world_base;
                if (laser_gt_path.poses.size() == 0) 
                {
                    q_world_ref_ini = q_world_ref;
                    t_world_ref_ini = t_world_ref;
                }
                Eigen::Quaterniond q_ref_ini_cur = q_world_ref_ini.inverse() * q_world_ref;
                Eigen::Vector3d t_ref_ini_cur = q_world_ref_ini.inverse() * (t_world_ref - t_world_ref_ini);

                nav_msgs::Odometry laser_gt_odom;
                laser_gt_odom.header.frame_id = "/world";
                laser_gt_odom.child_frame_id = "/gt";
                laser_gt_odom.header.stamp = ros::Time(cloud_time);
                laser_gt_odom.pose.pose.orientation.x = q_ref_ini_cur.x();
                laser_gt_odom.pose.pose.orientation.y = q_ref_ini_cur.y();
                laser_gt_odom.pose.pose.orientation.z = q_ref_ini_cur.z();
                laser_gt_odom.pose.pose.orientation.w = q_ref_ini_cur.w();
                laser_gt_odom.pose.pose.position.x = t_ref_ini_cur(0);
                laser_gt_odom.pose.pose.position.y = t_ref_ini_cur(1);
                laser_gt_odom.pose.pose.position.z = t_ref_ini_cur(2);

                geometry_msgs::PoseStamped laser_pose;
                laser_pose.header = laser_gt_odom.header;
                laser_pose.header.frame_id = "/world";
                laser_pose.pose = laser_gt_odom.pose.pose;
                laser_gt_path.header = laser_pose.header;
                laser_gt_path.poses.push_back(laser_pose);
                // std::cout << q_ref_ini_cur.coeffs().transpose() << std::endl;
                // std::cout << t_ref_ini_cur.transpose() << std::endl;
                bag.write("/current_odom", ros::Time(cloud_time), laser_gt_odom);
            }

            ros::Rate loop_rate(10);
            loop_rate.sleep();
        }
        else
            break;
    }

    return 0;
}
