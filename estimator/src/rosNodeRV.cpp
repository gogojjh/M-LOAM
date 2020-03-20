/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *
 * Reference:
 * Thread: http://www.runoob.com/w3cnote/cpp-std-thread.html
 *******************************************************/

#include <iostream>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <iomanip>

#include <pcl/common/common.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "common/common.hpp"
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/utility.h"
#include "utility/visualization.h"
#include "utility/cloud_visualizer.h"

using namespace std;

Estimator estimator;

// laser path groundtruth
nav_msgs::Path laser_gt_path;
ros::Publisher pub_laser_gt_path;
ros::Publisher pub_laser_gt_odom;
ros::Publisher pub_gps;
std::vector<ros::Publisher> pub_laser_cloud_list;

Pose pose_world_ref_ini;

bool b_pause = false;

int MLOAM_START_IDX;

void saveGroundTruth()
{
    if (MLOAM_RESULT_SAVE)
    {
        std::ofstream fout(MLOAM_GT_PATH.c_str(), std::ios::out);
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
}

void pauseCallback(std_msgs::StringConstPtr msg)
{
    printf("%s\n", msg->data.c_str());
    b_pause = !b_pause;
}

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        printf("please intput: rosrun mlod mlod_node_rv [config file] \n"
                "for example: "
                "rosrun mloam mloam_node_rv "
                "~/catkin_ws/src/M-LOAM/config/config_realvehicle.yaml" 
                "/Monster/dataset/UDI_dataset/xfl_20200301/short_test/"
                "1 /home/jjiao/trajectory_results/real_vehicle/rv01_20200301/\n");
        return 1;
    }
    ros::init(argc, argv, "mloam_node_rt");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    string data_path = argv[2];
    printf("read sequence: %s\n", argv[2]);

    MLOAM_RESULT_SAVE = std::stoi(argv[3]);
    OUTPUT_FOLDER = argv[4];
    MLOAM_ODOM_PATH = OUTPUT_FOLDER + "stamped_mloam_odom_estimate.txt";
    MLOAM_GT_PATH = OUTPUT_FOLDER + "stamped_mloam_gt_estimate.txt";
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "extrinsic_parameter.txt";
    EX_CALIB_EIG_PATH = OUTPUT_FOLDER + "calib_eig.txt";
    printf("save result (0/1): %d\n", MLOAM_RESULT_SAVE);
    if (MLOAM_RESULT_SAVE)
    {
        std::cout << "output path: " << OUTPUT_FOLDER << std::endl;
        std::remove(MLOAM_ODOM_PATH.c_str());
        std::remove(MLOAM_MAP_PATH.c_str());
        std::remove(EX_CALIB_RESULT_PATH.c_str());
        std::remove(EX_CALIB_EIG_PATH.c_str());
    }
    MLOAM_START_IDX = std::stoi(argv[5]);
    ROS_WARN("reading cloud ...");

    // *************************************
    // configuration
    readParameters(config_file);
    estimator.setParameter();
    registerPub(n);

    // set up subscriber and publisher 
    ros::Subscriber sub_pause = n.subscribe<std_msgs::String>("/mloam_pause", 5, pauseCallback);
    pub_laser_cloud_list.resize(NUM_OF_LASER);
    for (size_t i = 0; i < NUM_OF_LASER; i++) pub_laser_cloud_list[i] = n.advertise<sensor_msgs::PointCloud2>(CLOUD_TOPIC[i], 10);
    pub_laser_gt_odom = n.advertise<nav_msgs::Odometry>("/laser_gt_odom", 10);
    pub_laser_gt_path = n.advertise<nav_msgs::Path>("/laser_gt_path", 10);
    pub_gps = n.advertise<sensor_msgs::NavSatFix>("/novatel718d/pos", 10);

    std::thread cloud_visualizer_thread;
    if (PCL_VIEWER)
    {
        cloud_visualizer_thread = std::thread(&PlaneNormalVisualizer::Spin, &estimator.plane_normal_vis_);
    }

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

        // read GPS list
        file = std::fopen((data_path + "gps/timestamps.txt").c_str(), "r");
        if (!file)
        {
            printf("cannot find file: %sgps/timestamps.txt\n", data_path.c_str());
            ROS_BREAK();
            return 0;
        }
        double gps_time;
        vector<double> gps_time_list;
        while (fscanf(file, "%lf", &gps_time) != EOF)
        {
            gps_time_list.push_back(gps_time);
        }
        std::fclose(file);            

        // read GT list
        file = std::fopen((data_path + "gt_odom/timestamps.txt").c_str(), "r");
        if (!file)
        {
            printf("cannot find file: %sgt_odom/timestamps.txt\n", data_path.c_str());
            ROS_BREAK();
            return 0;
        }
        double gt_odom_time;
        vector<double> gt_odom_time_list;
        while (fscanf(file, "%lf", &gt_odom_time) != EOF)
        {
            gt_odom_time_list.push_back(gt_odom_time);
        }
        std::fclose(file);    
    }  

    // *************************************
    // read data
    for (size_t i = MLOAM_START_IDX; i < cloud_time_list.size(); i++)
    {	
		if (ros::ok())
		{
            // double cloud_time = cloud_time_list[i] - cloud_time_list[0] + base_time;
            double cloud_time = ros::Time::now().toSec();
            printf("process data: %d\n", i);
            stringstream ss;
            ss << setfill('0') << setw(6) << i;

            // load cloud
            printf("size of finding cloud ");
            std::vector<pcl::PointCloud<pcl::PointXYZ> > laser_cloud_list(NUM_OF_LASER);
            for (size_t j = 0; j < NUM_OF_LASER; j++)
            {
                stringstream ss_file;
                ss_file << "cloud_" << j << "/data/" << ss.str() << ".pcd";
                string cloud_path = data_path + ss_file.str();
                //printf("%lu  %f \n", i, cloud_time);
                // printf("%s\n", cloud_path.c_str());
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_path, laser_cloud_list[j]) == -1)
                {
                    printf("Couldn't read file %s\n", cloud_path.c_str());
                    ROS_BREAK();
                    return 0;
                }
                printf("%d ", laser_cloud_list[j].size());
                roiCloudFilter(laser_cloud_list[j], ROI_RANGE);
                sensor_msgs::PointCloud2 msg_cloud;
                pcl::toROSMsg(laser_cloud_list[j], msg_cloud);
                msg_cloud.header.frame_id = std::string("laser_") + std::to_string(j);
                msg_cloud.header.stamp = ros::Time(cloud_time);
                pub_laser_cloud_list[j].publish(msg_cloud);
            }
            printf("\n");

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
                pub_gps.publish(gps_position);
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
                Pose pose_world_base(q_world_base, t_world_base);
                Pose pose_base_ref(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
                Pose pose_world_ref(pose_world_base * pose_base_ref);
                if (laser_gt_path.poses.size() == 0) pose_world_ref_ini = pose_world_ref;
                Pose pose_ref_ini_cur(pose_world_ref_ini.inverse() * pose_world_ref);

                nav_msgs::Odometry laser_gt_odom;
                laser_gt_odom.header.frame_id = "/world";
                laser_gt_odom.child_frame_id = "/gt";
                laser_gt_odom.header.stamp = ros::Time(cloud_time);
                laser_gt_odom.pose.pose.orientation.x = pose_ref_ini_cur.q_.x();
                laser_gt_odom.pose.pose.orientation.y = pose_ref_ini_cur.q_.y();
                laser_gt_odom.pose.pose.orientation.z = pose_ref_ini_cur.q_.z();
                laser_gt_odom.pose.pose.orientation.w = pose_ref_ini_cur.q_.w();
                laser_gt_odom.pose.pose.position.x = pose_ref_ini_cur.t_(0);
                laser_gt_odom.pose.pose.position.y = pose_ref_ini_cur.t_(1);
                laser_gt_odom.pose.pose.position.z = pose_ref_ini_cur.t_(2);
                pub_laser_gt_odom.publish(laser_gt_odom);
                publishTF(laser_gt_odom);

                geometry_msgs::PoseStamped laser_gt_pose;
                laser_gt_pose.header.frame_id = "/world";
                laser_gt_pose.header.stamp = ros::Time(cloud_time);
                laser_gt_pose.pose = laser_gt_odom.pose.pose;
                laser_gt_path.header = laser_gt_pose.header;
                laser_gt_path.poses.push_back(laser_gt_pose);
                pub_laser_gt_path.publish(laser_gt_path);
                estimator.laser_gt_path_ = laser_gt_path;
            }

            estimator.inputCloud(cloud_time, laser_cloud_list);           

            ros::Duration duration(0.01);
            if (b_pause)
            {
                while (true)
                {
                    ros::spinOnce();
                    if ((!b_pause) || (!ros::ok())) break;
                    duration.sleep();
                }
            } else
            {
                ros::spinOnce();
                duration.sleep();
            }
		}
		else
			break;
	}

    // save gt (only once)
    // saveGroundTruth();
    return 0;
}

//
