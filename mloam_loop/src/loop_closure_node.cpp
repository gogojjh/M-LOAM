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

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "common/color.hpp"
#include "mloam_msgs/Keyframes.h"

#include "mloam_loop/parameters.hpp"
#include "mloam_loop/pose_graph.h"
#include "mloam_loop/keyframe.h"

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");

// loading parameter
int RESULT_SAVE;
std::string OUTPUT_FOLDER;
std::string MLOAM_LOOP_PATH;

// setting in config.yaml
int LOOP_SKIP_INTERVAL;
int LOOP_HISTORY_SEARCH_NUM;
double LOOP_DISTANCE_THRESHOLD;
double LOOP_OPTI_COST_THRESHOLD;
double LOOP_TEMPORAL_CONSISTENCY_THRESHOLD;
double LOOP_GLOBAL_REGISTRATION_THRESHOLD;
double LOOP_LOCAL_REGISTRATION_THRESHOLD;
int VISUALIZE_IMAGE;
int LOAD_PREVIOUS_POSE_GRAPH;
int LOOP_SAVE_PCD;

double LIDAR_HEIGHT;
int PC_NUM_RING;
int PC_NUM_SECTOR;
double PC_MAX_RADIUS;
double PC_UNIT_SECTORANGLE;
double PC_UNIT_RINGGAP;
int NUM_EXCLUDE_RECENT;
int NUM_CANDIDATES_FROM_TREE;
double SEARCH_RATIO;
double SC_DIST_THRES;
int TREE_MAKING_PERIOD;

double NORMAL_RADIUS;
double FPFH_RADIUS;
double DIV_FACTOR;
double USE_ABSOLUTE_SCALE;
double MAX_CORR_DIST;
double ITERATION_NUMBER;
double TUPLE_SCALE;
double TUPLE_MAX_CNT;

// setting in the main procedure
std::string POSE_GRAPH_SAVE_PATH;
int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;

std::mutex m_buf, m_process;

// input point cloud data
double time_laser_cloud_surf_last;
double time_laser_cloud_corner_last;
double time_laser_cloud_full_res;
double time_laser_cloud_outlier;
double time_laser_keyframes;

std::queue<sensor_msgs::PointCloud2ConstPtr> surf_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> corner_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> full_res_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> outlier_buf;
std::queue<mloam_msgs::KeyframesConstPtr> keyframes_buf;

pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf_last(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_last(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_full_res(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_outlier(new pcl::PointCloud<pcl::PointXYZI>());

int frame_cnt = 0;

PoseGraph posegraph;

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

void pubLoopInfo()
{
    // iscloam::LoopInfo loop;
    // loop.header.stamp = pointcloud_time;
    // loop.header.frame_id = "/laser";
    // loop.current_id = iscGeneration.current_frame_id;
    // for (int i = 0; i < (int)iscGeneration.matched_frame_id.size(); i++)
    // {
        // loop.matched_id.push_back(iscGeneration.matched_frame_id[i]);
    // }
    // loop_info_pub.publish(loop);
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
			m_buf.unlock();

            double time_keyframe = keyframes_msg.header.stamp.toSec();
            Eigen::Quaterniond q(keyframes_msg.poses.back().pose.pose.orientation.w,
                                 keyframes_msg.poses.back().pose.pose.orientation.x,
                                 keyframes_msg.poses.back().pose.pose.orientation.y,
                                 keyframes_msg.poses.back().pose.pose.orientation.z);
            Eigen::Vector3d t(keyframes_msg.poses.back().pose.pose.position.x,
                              keyframes_msg.poses.back().pose.pose.position.y,
                              keyframes_msg.poses.back().pose.pose.position.z);
            Pose pose_w(q, t);
            for (size_t i = 0; i < 6; i++)
                for (size_t j = 0; j < 6; j++)
                    pose_w.cov_(i, j) = double(keyframes_msg.poses.back().pose.covariance[i * 6 + j]);

            KeyFrame *keyframe = new KeyFrame(time_keyframe,
                                              frame_cnt,
                                              pose_w,
                                              laser_cloud_surf_last,
                                              laser_cloud_corner_last,
                                              laser_cloud_full_res,
                                              laser_cloud_outlier,
                                              0);

            m_process.lock();
            posegraph.skip_cnt_++;
            if (posegraph.skip_cnt_ >= LOOP_SKIP_INTERVAL)
            {
                printf("start loop detection: %d\n", posegraph.skip_cnt_);
                posegraph.addKeyFrame(keyframe, 1);
            }
            else
            {
                printf("skip loop detection: %d\n", posegraph.skip_cnt_);
                posegraph.addKeyFrame(keyframe, 0);
            }
            m_process.unlock();
            frame_cnt++;            
            std::cout << common::RED << "keyframe size: " << posegraph.getKeyFrameSize() << common::RESET << std::endl << std::endl;
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void sigintHandler(int sig)
{
    printf("[loop_closure_node] press ctrl-c\n");
    // std::cout << common::YELLOW << "mapping drop frame: " << frame_drop_cnt << common::RESET << std::endl;
    if (RESULT_SAVE)
    {
        m_process.lock();
        posegraph.savePoseGraph();
        m_process.unlock();
    }
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
    posegraph.registerPub(nh);

    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;

    RESULT_SAVE = FLAGS_result_save;
    OUTPUT_FOLDER = FLAGS_output_path;
    MLOAM_LOOP_PATH = OUTPUT_FOLDER + "traj/stamped_mloam_loop_estimate.txt";
    POSE_GRAPH_SAVE_PATH = OUTPUT_FOLDER + "pose_graph/";
    printf("[loop_closure_node] save result (0/1): %d to %s\n", RESULT_SAVE, OUTPUT_FOLDER.c_str());
    printf("config_file: %s\n", FLAGS_config_file.c_str());

    cv::FileStorage fsSettings(FLAGS_config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }


    LOOP_SKIP_INTERVAL = fsSettings["loop_skip_interval"];
    LOOP_HISTORY_SEARCH_NUM = fsSettings["loop_history_search_num"];
    LOOP_DISTANCE_THRESHOLD = fsSettings["loop_distance_threshold"];
    LOOP_TEMPORAL_CONSISTENCY_THRESHOLD = fsSettings["loop_temporal_consistency_threshold"];
    LOOP_GLOBAL_REGISTRATION_THRESHOLD = fsSettings["loop_global_registration_threshold"];
    LOOP_LOCAL_REGISTRATION_THRESHOLD = fsSettings["loop_local_registration_threshold"];
    VISUALIZE_IMAGE = fsSettings["visualize_image"];
    LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
    LOOP_SAVE_PCD = fsSettings["loop_save_pcd"];

    // scan context
    LIDAR_HEIGHT = fsSettings["lidar_height"];
    PC_NUM_RING = fsSettings["pc_num_ring"];
    PC_NUM_SECTOR = fsSettings["pc_num_sector"];
    PC_MAX_RADIUS = fsSettings["pc_max_radius"];
    PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);
    NUM_EXCLUDE_RECENT = fsSettings["num_exclude_recent"];
    NUM_CANDIDATES_FROM_TREE = fsSettings["num_candidates_from_tree"];
    SEARCH_RATIO = fsSettings["search_ratio"];
    SC_DIST_THRES = fsSettings["sc_dist_thres"];
    TREE_MAKING_PERIOD = fsSettings["tree_making_period"];

    // registration
    NORMAL_RADIUS = fsSettings["normal_radius"];
    FPFH_RADIUS = fsSettings["fpfh_radius"];
    DIV_FACTOR = fsSettings["div_factor"];
    USE_ABSOLUTE_SCALE = fsSettings["use_absolute_scale"];
    MAX_CORR_DIST = fsSettings["max_corr_dist"];
    ITERATION_NUMBER = fsSettings["iteration_number"];
    TUPLE_SCALE = fsSettings["tuple_scale"];
    TUPLE_MAX_CNT = fsSettings["tuple_max_cnt"];

    fsSettings.release();

    posegraph.setParameter();
    posegraph.setPGOTread();
    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        // printf("Load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        // printf("Load pose graph finish\n");
    }
    else
    {
        printf("Not load pose graph\n");
    }

    // *******************************
    ros::Subscriber sub_laser_cloud_full_res = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud", 2, laserCloudFullResHandler);
    ros::Subscriber sub_laser_cloud_outlier = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_outlier", 2, laserCloudOutlierResHandler);
    ros::Subscriber sub_laser_cloud_surf_last = nh.subscribe<sensor_msgs::PointCloud2>("/surf_points_less_flat", 2, laserCloudSurfLastHandler);
	ros::Subscriber sub_laser_cloud_corner_last = nh.subscribe<sensor_msgs::PointCloud2>("/corner_points_less_sharp", 2, laserCloudCornerLastHandler);
    ros::Subscriber sub_laser_keyframes = nh.subscribe<mloam_msgs::Keyframes>("/laser_map_keyframes_6d", 5, laserKeyframeHandler);

    // pub_laser_loop_keyframes_6d = nh.advertise<mloam_msgs::Keyframes>("/laser_loop_keyframes_6d", 5);
    // pub_scan_context = nh.advertise<sensor_msgs::Image>("/input_scan_context", 5);

    signal(SIGINT, sigintHandler);
    std::thread loop_closure_node_thread{process};
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
