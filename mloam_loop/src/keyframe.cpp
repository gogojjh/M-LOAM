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

#include "mloam_loop/keyframe.h"

// create keyframe online
KeyFrame::KeyFrame(const double &time_stamp,
				   const int &index,
				   const Pose &pose_w,
				   const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_cloud,
				   const pcl::PointCloud<pcl::PointXYZI>::Ptr &corner_cloud,
				   const pcl::PointCloud<pcl::PointXYZI>::Ptr &full_cloud,
				   const pcl::PointCloud<pcl::PointXYZI>::Ptr &outlier_cloud,
				   const int sequence)
{
	time_stamp_ = time_stamp;
	index_ = index;
	pose_w_ = pose_w;
	pose_3d_w_.x = pose_w.t_(0);
	pose_3d_w_.y = pose_w.t_(1);
	pose_3d_w_.z = pose_w.t_(2);
	surf_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	corner_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	full_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	outlier_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	*surf_cloud_ = *surf_cloud;
	*corner_cloud_ = *corner_cloud;
	*full_cloud_ = *full_cloud;
	*outlier_cloud_ = *outlier_cloud;
	has_loop_ = false;
	loop_index_ = -1;
	loop_info_ = Pose(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
	sequence_ = sequence;
}

// load previous keyframe
KeyFrame::KeyFrame(const double &time_stamp,
				   const int &index,
				   const Pose &pose_w,
				   const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_cloud,
				   const pcl::PointCloud<pcl::PointXYZI>::Ptr &corner_cloud,
				   const pcl::PointCloud<pcl::PointXYZI>::Ptr &full_cloud,
				   const pcl::PointCloud<pcl::PointXYZI>::Ptr &outlier_cloud,
				   const int &loop_index,
				   const Pose &loop_info,
				   const int sequence)
{
	time_stamp_ = time_stamp;
	index_ = index;
	pose_w_ = pose_w;
	pose_3d_w_.x = pose_w.t_(0);
	pose_3d_w_.y = pose_w.t_(1);
	pose_3d_w_.z = pose_w.t_(2);
	surf_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	corner_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	full_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	outlier_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	*surf_cloud_ = *surf_cloud;
	*corner_cloud_ = *corner_cloud;
	*full_cloud_ = *full_cloud;
	*outlier_cloud_ = *outlier_cloud;
	if (loop_index != -1)
		has_loop_ = true;
	else
		has_loop_ = false;
	loop_index_ = loop_index;
	loop_info_ = loop_info;
	sequence_ = sequence;
}

void KeyFrame::getPose(Pose &pose_w)
{
	pose_w = pose_w_;
}

void KeyFrame::getLastPose(Pose &last_pose_w)
{
	last_pose_w = last_pose_w_;
}

void KeyFrame::updatePose(const Pose &pose_w)
{
	last_pose_w_ = pose_w_;
    pose_w_ = pose_w;
}

void KeyFrame::updateLoopInfo(const int loop_index, const Pose &loop_info)
{
	has_loop_ = true;
	loop_index_ = loop_index;
	loop_info_ = loop_info;
}

Pose KeyFrame::getLoopRelativePose()
{
	return loop_info_;
}

// void KeyFrame::updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info)
// {
// 	if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
// 	{
// 		//printf("update loop info\n");
// 		loop_info = _loop_info;
// 	}
// }

