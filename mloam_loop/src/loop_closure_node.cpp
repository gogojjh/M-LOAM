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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "common/color.hpp"

#include "mloam_msgs/Keyframes.h"

#include "mloam_loop/tic_toc.h"
#include "mloam_loop/parameters.h"
#include "mloam_loop/pose.h"
#include "mloam_loop/scan_context.hpp"

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

// ros
ros::Publisher pub_laser_loop_keyframes_6d;
ros::Publisher pub_scan_context;

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

// store keyframes 
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_keyframes_3d(new pcl::PointCloud<pcl::PointXYZI>());
std::vector<std::pair<double, Pose> > laser_keyframes_6d;
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> surf_keyframes;
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> corner_keyframes;
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_keyframes;
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outlier_keyframes;

// loop result
int cur_kf_idx = 0; 
int loop_kf_idx = 0;
float yaw_diff_rad = 0.0;

// store map and current cloud
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf_from_map(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_from_map(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_surf_from_map_ds(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_corner_from_map_ds(new pcl::PointCloud<pcl::PointXYZI>());
pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_surf_map;
pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_corner_map;

// others
size_t frame_cnt = 0; // frame num
size_t frame_drop_cnt = 0; // drop keyframes for real-time performance
size_t frame_skip_cnt = 0; // skip performing loop detection

// scancontext
SCManager sc_manager;

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

void addNewKeyframes(const mloam_msgs::Keyframes &keyframes_msg)
{
    double time_keyframe = keyframes_msg.header.stamp.toSec();
    Eigen::Quaterniond q(keyframes_msg.poses.back().pose.pose.orientation.w,
                         keyframes_msg.poses.back().pose.pose.orientation.x,
                         keyframes_msg.poses.back().pose.pose.orientation.y,
                         keyframes_msg.poses.back().pose.pose.orientation.z);
    Eigen::Vector3d t(keyframes_msg.poses.back().pose.pose.position.x,
                      keyframes_msg.poses.back().pose.pose.position.y,
                      keyframes_msg.poses.back().pose.pose.position.z);
    Pose pose(q, t);
    for (size_t i = 0; i < 6; i++)
        for (size_t j = 0; j < 6; j++)
            pose.cov_(i, j) = double(keyframes_msg.poses.back().pose.covariance[i * 6 + j]);

    pcl::PointXYZI pose_3d;
    pose_3d.x = pose.t_[0];
    pose_3d.y = pose.t_[1];
    pose_3d.z = pose.t_[2];
    pose_3d.intensity = laser_keyframes_3d->size();
    laser_keyframes_3d->push_back(pose_3d);
    laser_keyframes_6d.push_back(std::make_pair(time_keyframe, pose));

    surf_keyframes.push_back(laser_cloud_surf_last);
    corner_keyframes.push_back(laser_cloud_corner_last);
    cloud_keyframes.push_back(laser_cloud_full_res);
    outlier_keyframes.push_back(laser_cloud_outlier);

    cur_kf_idx = laser_keyframes_6d.size();
    std::cout << "keyframe size: " << cur_kf_idx << std::endl;
}

bool loopDetection()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *laser_cloud += *cloud_keyframes[cur_kf_idx];
    *laser_cloud += *outlier_keyframes[cur_kf_idx];
    sc_manager.makeAndSaveScancontextAndKeys(*laser_cloud);

    // 1. check if skip enough frame
    frame_skip_cnt++;
    if (frame_skip_cnt <= LOOP_KEYFRAME_INTERVAL) return false;

    // 2. apply scan context-based global localization
    auto detect_result = sc_manager.detectLoopClosureID();
    // std::cout << "match id: " << detect_result.first 
    //           << ", yaw: " << detect_result.second << std::endl;   
    loop_kf_idx = detect_result.first;
    yaw_diff_rad = detect_result.second;
    if (loop_kf_idx == -1) return false;

    // TODO: check if the loop_kf is too far from the cur_kf

    // 3. construct the current point cloud
    pcl::PointCloud<pcl::PointXYZI> surf_trans, corner_trans;

    laser_cloud_surf->clear();
    laser_cloud_corner->clear();
    pcl::transformPointCloud(*surf_keyframes[cur_kf_idx],
                             surf_trans,
                             laser_keyframes_6d[cur_kf_idx].second.T_.cast<float>());
    *laser_cloud_surf += surf_trans;

    pcl::transformPointCloud(*corner_keyframes[cur_kf_idx],
                             corner_trans,
                             laser_keyframes_6d[cur_kf_idx].second.T_.cast<float>());
    *laser_cloud_corner += corner_trans;

    // construct the model point cloud
    laser_cloud_surf_from_map->clear();
    laser_cloud_corner_from_map->clear();
    for (int j = -LOOP_HISTORY_SEARCH_NUM; j <= LOOP_HISTORY_SEARCH_NUM; j++)
    {
        if (loop_kf_idx + j < 0 || loop_kf_idx + j > cur_kf_idx) continue;
        pcl::transformPointCloud(*surf_keyframes[loop_kf_idx + j],
                                 surf_trans,
                                 laser_keyframes_6d[loop_kf_idx + j].second.T_.cast<float>());
        *laser_cloud_surf_from_map += surf_trans;

        pcl::transformPointCloud(*corner_keyframes[loop_kf_idx + j],
                                 corner_trans,
                                 laser_keyframes_6d[loop_kf_idx + j].second.T_.cast<float>());
        *laser_cloud_corner_from_map += corner_trans;
    }
    down_size_filter_surf_map.setInputCloud(laser_cloud_surf_from_map);
    down_size_filter_surf_map.filter(*laser_cloud_surf_from_map_ds);
    down_size_filter_corner_map.setInputCloud(laser_cloud_corner_from_map);
    down_size_filter_corner_map.filter(*laser_cloud_corner_from_map_ds);

    frame_skip_cnt = 0;
    return true;
}

// find relative transformation from map to local
std::pair<double, Pose> performLoopOptimization()
{
    // optimize laser_cloud_surf <-> laser_clous_surf_from_map_ds
    // optimize laser_cloud_corner <-> laser_clous_corner_from_map_ds
    size_t laser_cloud_surf_from_map_num = laser_cloud_surf_from_map_ds->size();
    size_t laser_cloud_corner_from_map_num = laser_cloud_corner_from_map_ds->size();
    printf("[loop_closure] map surf num: %lu, corner num: %lu\n", laser_cloud_surf_from_map_num, laser_cloud_corner_from_map_num);
    kdtree_surf_from_map->setInputCloud(laser_cloud_surf_from_map_ds);
    kdtree_corner_from_map->setInputCloud(laser_cloud_corner_from_map_ds);
    double opti_cost = 300;

    Pose pose_local;
    for (int iter_cnt = 0; iter_cnt < 2; iter_cnt++)
    {
        double *para_pose = new double[SIZE_POSE];
        ceres::Problem problem;
        ceres::LossFunctionWrapper *loss_function = new ceres::LossFunctionWrapper(new ceres::HuberLoss(1), ceres::TAKE_OWNERSHIP);

        para_pose[0] = pose_local.t_(0);
        para_pose[1] = pose_local.t_(1);
        para_pose[2] = pose_local.t_(2);
        para_pose[3] = pose_local.q_.x();
        para_pose[4] = pose_local.q_.y();
        para_pose[5] = pose_local.q_.z();
        para_pose[6] = pose_local.q_.w();

        std::vector<PointPlaneFeature> all_surf_features, all_corner_features;
        size_t surf_num = 0, corner_num = 0;
        TicToc t_match_features;
        int n_neigh = 5;
        f_extract_.matchSurfFromMap(kdtree_surf_points_local_map,
                                    *laser_cloud_surf_from_map_ds,
                                    *laser_cloud_surf,
                                    pose_local,
                                    all_surf_features,
                                    n_neigh,
                                    false);
        surf_num = all_surf_features.size();

        f_extract_.matchCornerFromMap(kdtree_corner_points_local_map,
                                      *laser_cloud_corner_from_map_ds,
                                      *laser_cloud_corner,
                                      pose_local,
                                      all_corner_features,
                                      n_neigh,
                                      false);
        corner_num = all_corner_features.size();
        printf("matching features time: %fms\n", t_match_features.toc());

        for (const PointPlaneFeature &feature : all_surf_features)
        {
            Eigen::Matrix3d cov_matrix;
            cov_matrix.setIdentity();
            LidarMapPlaneNormFactor *f = new LidarMapPlaneNormFactor(feature.point_, feature.coeffs_, cov_matrix);
            problem.AddResidualBlock(f, loss_function, para_pose);
        }
        for (const PointPlaneFeature &feature : all_corner_features)
        {
            Eigen::Matrix3d cov_matrix;
            cov_matrix.setIdentity();
            LidarMapPlaneNormFactor *f = new LidarMapPlaneNormFactor(feature.point_, feature.coeffs_, cov_matrix);
            problem.AddResidualBlock(f, loss_function, para_pose);
        }

        TicToc t_solver;
        ceres::Solver::Summary summary;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        options.max_num_iterations = 30;
        options.max_solver_time_in_seconds = 0.03;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        opti_cost = std::min((double)summary.final_cost, opti_cost);   
        printf("solver time: %fms\n", t_solver.toc());

        pose_local.t_ = Eigen::Vector3d(para_pose[0], para_pose[1], para_pose[2]);
        pose_local.q_ = Eigen::Quaterniond(para_pose[6], para_pose[3], para_pose[4], para_pose[5]);
        pose_local.update();
    }
    std::pair<double, Pose> loop_opti_result = make_pair(total_cost, pose_local);
    std::cout << "relative transformation: " << pose_local << std::endl;  
    return loop_opti_result;
}

void pubLoopInfo()
{
    cv::Mat sc_img = sc_manager.getLastScanContextImage();
    cv_bridge::CvImage last_scan_context_msg;
    last_scan_context_msg.header.frame_id = "/laser";
    last_scan_context_msg.header.stamp = ros::Time().fromSec(time_laser_keyframes);
    last_scan_context_msg.encoding = sensor_msgs::image_encodings::RGB8;
    last_scan_context_msg.image = sc_img;
    pub_scan_context.publish(last_scan_context_msg.toImageMsg());

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

            // TODO: depend on the calculation speed
    		// while (!keyframes_buf.empty())
            // {
			// 	keyframes_buf.pop();
            //     frame_drop_cnt++;
            //     std::cout << common::GREEN << "[loop_closure] drop keyframe real time performance" << common::RESET << std::endl;
            // }
			m_buf.unlock();

            // main process
            std::lock_guard<std::mutex> lock(m_process);
            frame_cnt++;
            TicToc t_loop_closure;

            // step 1: add new keyframes
            addNewKeyframes(keyframes_msg);

            // step 2: detect loop candidates
            TicToc t_loop_detect;
            bool detect_loop = loopDetection();
            printf("loop_detection: %fms\n", t_loop_detect.toc());

            if (detect_loop)
            {
                printf("find loop: cur_kf_idx: %d, loop_kf_idx: %d\n", cur_kf_idx, loop_kf_idx);
                // consistencyVerfication();

                auto loop_opti_result = performLoopOptimization();
                if (loop_opti_result.first > LOOP_OPTI_COST_THRESHOLD)
                {
                    printf("loop reject with geometry verificiation\n");
                    continue;
                } 
                Pose pose_rlt = loop_opti_result.second;

                // addPoseGraph();
                // poseGraphOptimization();
            } 
            else
            {

            }

            pubLoopInfo();

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
    std::cout << common::YELLOW << "mapping drop frame: " << frame_drop_cnt << common::RESET << std::endl;
    if (RESULT_SAVE)
    {
        // save_statistics.saveMapStatistics(MLOAM_MAP_PATH,
        //                                   OUTPUT_FOLDER + "others/mapping_factor.txt",
        //                                   OUTPUT_FOLDER + "others/mapping_d_eigvec.txt",
        //                                   OUTPUT_FOLDER + "others/mapping_sp_" + FLAGS_gf_method + "_" + std::to_string(FLAGS_gf_ratio_ini) + ".txt",
        //                                   OUTPUT_FOLDER + "others/mapping_logdet_H.txt",
        //                                   laser_after_mapped_path,
        //                                   d_factor_list,
        //                                   d_eigvec_list,
        //                                   mapping_sp_list,
        //                                   logdet_H_list);
        // save_statistics.saveMapTimeStatistics(OUTPUT_FOLDER + "time/time_mloam_mapping_" + FLAGS_gf_method + "_" + std::to_string(FLAGS_gf_ratio_ini) + "_" + FLAGS_loss_mode + "_" + std::to_string(int(FLAGS_gnc)) + ".txt",
        //                                       total_match_feature,
        //                                       total_solver,
        //                                       total_mapping);
    }
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

    sc_manager.setParameter(LIDAR_HEIGHT,
                            PC_NUM_RING,
                            PC_NUM_SECTOR,
                            PC_MAX_RADIUS,
                            PC_UNIT_SECTORANGLE,
                            PC_UNIT_RINGGAP,
                            NUM_EXCLUDE_RECENT,
                            NUM_CANDIDATES_FROM_TREE,
                            SEARCH_RATIO,
                            SC_DIST_THRES,
                            TREE_MAKING_PERIOD);

    ros::Subscriber sub_laser_cloud_full_res = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud", 2, laserCloudFullResHandler);
    ros::Subscriber sub_laser_cloud_outlier = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_outlier", 2, laserCloudOutlierResHandler);
    ros::Subscriber sub_laser_cloud_surf_last = nh.subscribe<sensor_msgs::PointCloud2>("/surf_points_less_flat", 2, laserCloudSurfLastHandler);
	ros::Subscriber sub_laser_cloud_corner_last = nh.subscribe<sensor_msgs::PointCloud2>("/corner_points_less_sharp", 2, laserCloudCornerLastHandler);
    ros::Subscriber sub_laser_keyframes = nh.subscribe<mloam_msgs::Keyframes>("/laser_map_keyframes_6d", 5, laserKeyframeHandler);

    pub_laser_loop_keyframes_6d = nh.advertise<mloam_msgs::Keyframes>("/laser_loop_keyframes_6d", 5);
    pub_scan_context = nh.advertise<sensor_msgs::Image>("/input_scan_context", 5);
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
