/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "parameters.hpp"
#include "utility/tic_toc.h"
#include "utility/pose.h"

class KeyFrame
{
public:
	KeyFrame(const double &time_stamp,
			 const int &index,
			 const Pose &pose_w,
			 const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_cloud,
			 const pcl::PointCloud<pcl::PointXYZI>::Ptr &corner_cloud,
			 const pcl::PointCloud<pcl::PointXYZI>::Ptr &full_cloud,
			 const pcl::PointCloud<pcl::PointXYZI>::Ptr &outlier_cloud,
			 const int sequence);

	KeyFrame(const double &time_stamp,
			 const int &index,
			 const Pose &pose_w,
			 const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_cloud,
			 const pcl::PointCloud<pcl::PointXYZI>::Ptr &corner_cloud,
			 const pcl::PointCloud<pcl::PointXYZI>::Ptr &full_cloud,
			 const pcl::PointCloud<pcl::PointXYZI>::Ptr &outlier_cloud,
			 const int &loop_index,
			 const Pose &loop_info,
			 const int sequence);

	void getPose(Pose &pose_w);
	void getLastPose(Pose &last_pose_w);
	void updatePose(const Pose &pose_w);
	void updateLoopInfo(const int loop_index, const Pose &loop_info);
	Pose getLoopRelativePose();

	double time_stamp_;
	int index_;
	int local_index_;
	Pose pose_w_;
	Pose last_pose_w_;
	pcl::PointXYZI pose_3d_w_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr outlier_cloud_;

	bool has_loop_;
	int loop_index_;
	Pose loop_info_;

	int sequence_;
};

