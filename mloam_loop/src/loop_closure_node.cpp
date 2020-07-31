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
#include <memory>
#include <mutex>
#include <queue>
#include <stack>
#include <thread>
#include <signal.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/color.hpp"

#include "mloam_msgs/Keyframes.h"

#include "mloam_loop/tic_toc.h"
#include "mloam_loop/parameters.h"
#include "mloam_loop/pose.h"

//my library
// #include "iscOptimizationClass.h"
// #include <iscloam/LoopInfo.h>

// ros::Publisher odom_pub;
// //ros::Publisher map_pub;
// ros::Publisher path_pub;
// ros::Publisher loop_map_pub;
// ros::Publisher loop_candidate_pub;

// ISCOptimizationClass iscOptimization;

// double scan_period= 0.1;

// Eigen::Isometry3d w_odom_curr= Eigen::Isometry3d::Identity();
// nav_msgs::Path path_optimized;
// int current_frame_id;
// std::vector<int> matched_frame_id;
//receive odomtry

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");

std::mutex m_buf, m_process;

std::queue<sensor_msgs::PointCloud2ConstPtr> surf_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> corner_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> full_res_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> outlier_buf;
std::queue<mloam_msgs::KeyframesConstPtr> keyframes_buf;

pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf_last(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_last(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_full_res(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_outlier(new pcl::PointCloud<pcl::PointXYZI>());

double time_laser_cloud_surf_last;
double time_laser_cloud_corner_last;
double time_laser_cloud_full_res;
double time_laser_cloud_outlier;
double time_laser_keyframes;

size_t cur_keyframes_size = 0;
std::vector<std::pair<double, Pose> > laser_keyframes_6d;

ros::Publisher pub_laser_loop_keyframes_6d;

size_t frame_cnt = 0;

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_surf_last_msg)
{
	m_buf.lock();
	surf_last_buf.push(laser_cloud_surf_last_msg);
	m_buf.unlock();
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_corner_last_msg)
{
	m_buf.lock();
	corner_last_buf.push(laser_cloud_corner_last_msg);
	m_buf.unlock();
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_full_res_msg)
{
	m_buf.lock();
	full_res_buf.push(laser_cloud_full_res_msg);
	m_buf.unlock();
}

void laserCloudOutlierResHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_outlier_msg)
{
    m_buf.lock();
    outlier_buf.push(laser_cloud_outlier_msg);
    m_buf.unlock();
}

void laserKeyframeHandler(const mloam_msgs::KeyframesConstPtr &laser_key_frames_msg)
{
    m_buf.lock();
    keyframes_buf.push(laser_key_frames_msg);
    m_buf.unlock();
}

void process()
{
    while(1)
    {
		if (!ros::ok()) break;
		while (!surf_last_buf.empty() && !corner_last_buf.empty() &&
			   !full_res_buf.empty() && !outlier_buf.empty() && !keyframes_buf.empty())
		{
			m_buf.lock();
			while (!surf_last_buf.empty() && surf_last_buf.front()->header.stamp.toSec() < keyframes_buf.front()->header.stamp.toSec())
				surf_last_buf.pop();
			if (surf_last_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			while (!corner_last_buf.empty() && corner_last_buf.front()->header.stamp.toSec() < keyframes_buf.front()->header.stamp.toSec())
				corner_last_buf.pop();
			if (corner_last_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			while (!full_res_buf.empty() && full_res_buf.front()->header.stamp.toSec() < keyframes_buf.front()->header.stamp.toSec())
				full_res_buf.pop();
			if (full_res_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			while (!outlier_buf.empty() && outlier_buf.front()->header.stamp.toSec() < keyframes_buf.front()->header.stamp.toSec())
				outlier_buf.pop();
			if (outlier_buf.empty())
			{
				m_buf.unlock();
				break;
			}    

			time_laser_cloud_surf_last = surf_last_buf.front()->header.stamp.toSec();
			time_laser_cloud_corner_last = corner_last_buf.front()->header.stamp.toSec();
			time_laser_cloud_full_res = full_res_buf.front()->header.stamp.toSec();
            time_laser_cloud_outlier = outlier_buf.front()->header.stamp.toSec();
			time_laser_keyframes = keyframes_buf.front()->header.stamp.toSec();

            if (std::abs(time_laser_keyframes - time_laser_cloud_surf_last) > 0.005 ||
                std::abs(time_laser_keyframes - time_laser_cloud_corner_last) > 0.005 ||
                std::abs(time_laser_keyframes - time_laser_cloud_full_res) > 0.005 ||
                std::abs(time_laser_keyframes - time_laser_cloud_outlier) > 0.005)
            {
                printf("time surf: %f, corner: %f, full: %f, outlier: %f, keyframes: %f\n",
                       time_laser_cloud_surf_last, 
                       time_laser_cloud_corner_last,
                       time_laser_cloud_full_res, 
                       time_laser_cloud_outlier,
                       time_laser_keyframes);
                printf("unsync messeage!");
				m_buf.unlock();
				break;
			}

			laser_cloud_surf_last->clear();
			pcl::fromROSMsg(*surf_last_buf.front(), *laser_cloud_surf_last);
			surf_last_buf.pop();

			laser_cloud_corner_last->clear();
			pcl::fromROSMsg(*corner_last_buf.front(), *laser_cloud_corner_last);
			corner_last_buf.pop();

			laser_cloud_full_res->clear();
			pcl::fromROSMsg(*full_res_buf.front(), *laser_cloud_full_res);
			full_res_buf.pop();

            laser_cloud_outlier->clear();
            pcl::fromROSMsg(*outlier_buf.front(), *laser_cloud_outlier);
            outlier_buf.pop();

            mloam_msgs::Keyframes keyframes_msg = *keyframes_buf.front();
            keyframes_buf.pop();
            laser_keyframes_6d.clear();
            laser_keyframes_6d.resize(keyframes_msg.poses.size());
            for (size_t k = 0; k < keyframes_msg.poses.size(); k++)
            {
                double time_keyframe = keyframes_msg.poses[k].header.stamp.toSec();
                Eigen::Quaterniond q(keyframes_msg.poses[k].pose.pose.orientation.w,
                                     keyframes_msg.poses[k].pose.pose.orientation.x,
                                     keyframes_msg.poses[k].pose.pose.orientation.y,
                                     keyframes_msg.poses[k].pose.pose.orientation.z);
                Eigen::Vector3d t(keyframes_msg.poses[k].pose.pose.position.x,
                                  keyframes_msg.poses[k].pose.pose.position.y,
                                  keyframes_msg.poses[k].pose.pose.position.z);
                Pose pose(q, t);
                for (size_t i = 0; i < 6; i++)
                    for (size_t j = 0; j < 6; j++)
                        pose.cov_(i, j) = double(keyframes_msg.poses[i].pose.covariance[i * 6 + j]);
                laser_keyframes_6d[k] = std::make_pair(time_keyframe, pose);
            }

    		while (!keyframes_buf.empty())
            {
				keyframes_buf.pop();
			}
			m_buf.unlock();

            // main process
            std::lock_guard<std::mutex> lock(m_process);
            frame_cnt++;
            TicToc t_loop_closure;

            std::cout << "keyframe size: " << laser_keyframes_6d.size() << std::endl;

            // loopDetection();

            // consistencyVerfication();

            // buildConstraint();

            // addPoseGraph();

            // poseGraphOptimization();

            std::cout << common::RED << "frame: " << frame_cnt
                      << ", loop closure time: " << t_loop_closure.toc() << "ms" << common::RESET << std::endl;
            printf("\n");
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void sigintHandler(int sig)
{
    printf("[loop_closure_node] press ctrl-c\n");
    // std::cout << common::YELLOW << "mapping drop frame: " << frame_drop_cnt << common::RESET << std::endl;
    // if (MLOAM_RESULT_SAVE)
    // {
    //     save_statistics.saveMapStatistics(MLOAM_MAP_PATH,
    //                                       OUTPUT_FOLDER + "others/mapping_factor.txt",
    //                                       OUTPUT_FOLDER + "others/mapping_d_eigvec.txt",
    //                                       OUTPUT_FOLDER + "others/mapping_sp_" + FLAGS_gf_method + "_" + std::to_string(FLAGS_gf_ratio_ini) + ".txt",
    //                                       OUTPUT_FOLDER + "others/mapping_logdet_H.txt",
    //                                       laser_after_mapped_path,
    //                                       d_factor_list,
    //                                       d_eigvec_list,
    //                                       mapping_sp_list,
    //                                       logdet_H_list);
    //     save_statistics.saveMapTimeStatistics(OUTPUT_FOLDER + "time/time_mloam_mapping_" + FLAGS_gf_method + "_" + std::to_string(FLAGS_gf_ratio_ini) + "_" + FLAGS_loss_mode + "_" + std::to_string(int(FLAGS_gnc)) + ".txt",
    //                                           total_match_feature,
    //                                           total_solver,
    //                                           total_mapping);
    // }
    // saveGlobalMap();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("please intput: rosrun mloam_loop loop_closure_node -help\n");
        return 1;
    }
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "loop_closure_node");
    ros::NodeHandle nh("~");

    RESULT_SAVE = FLAGS_result_save;
    OUTPUT_FOLDER = FLAGS_output_path;
    printf("[loop_closure_node] save result (0/1): %d to %s\n", RESULT_SAVE, OUTPUT_FOLDER.c_str());

    printf("config_file: %s\n", FLAGS_config_file.c_str());
    readParameters(FLAGS_config_file);

	ros::Subscriber sub_laser_cloud_full_res = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud", 2, laserCloudFullResHandler);
    ros::Subscriber sub_laser_cloud_outlier = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_outlier", 2, laserCloudOutlierResHandler);
    ros::Subscriber sub_laser_cloud_surf_last = nh.subscribe<sensor_msgs::PointCloud2>("/surf_points_less_flat", 2, laserCloudSurfLastHandler);
	ros::Subscriber sub_laser_cloud_corner_last = nh.subscribe<sensor_msgs::PointCloud2>("/corner_points_less_sharp", 2, laserCloudCornerLastHandler);
    ros::Subscriber sub_laser_keyframes = nh.subscribe<mloam_msgs::Keyframes>("/laser_map_keyframes_6d", 5, laserKeyframeHandler);

    pub_laser_loop_keyframes_6d = nh.advertise<mloam_msgs::Keyframes>("/laser_loop_keyframes_6d", 5);
    // loop_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/loop_map", 100);
    // loop_candidate_pub = nh.advertise<sensor_msgs::PointCloud2>("/loop_candidate", 100);

    // int sector_width =60;
    // int ring_height = 60;
    // double max_distance= 40.0;

    // nh.getParam("/sector_width", sector_width); 
    // nh.getParam("/ring_height", ring_height); 
    // nh.getParam("/max_distance", max_distance);  
    // nh.getParam("/scan_period", scan_period); 

    // //init ISC
    // iscOptimization.init();

    // //open thread
    // std::thread global_optimization_process{global_optimization};

    signal(SIGINT, sigintHandler);

    std::thread loop_closure_node_thread{process};

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    loop_closure_node_thread.join();    
    return 0;
}
