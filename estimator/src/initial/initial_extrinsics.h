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

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <ros/console.h>

#include "common/types/type.h"
#include "common/algos/math.hpp"

#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialExtrinsics
{
public:
	InitialExtrinsics();
	void clearState();
	void setParameter();

	void addPose(const Pose &pose, const size_t &idx);

	bool calibExRotation(const size_t &idx_ref, const size_t &idx_data, Pose &calib_result);
	void calibExTranslation(const size_t &idx_ref, const size_t &idx_data);
	void calibExTranslationPlanar(const size_t &idx_ref, const size_t &idx_data);
	void calibTimeDelay(const size_t &idx_ref, const size_t &idx_data);

	bool setCovRotation(const size_t &idx);
	bool checkScrewMotion(const Pose &pose_ref, const Pose &pose_data);
	void saveStatistics();

	void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2, cv::Mat_<double> &t1, cv::Mat_<double> &t2);

	std::vector<Pose> calib_ext_;

	std::vector<double> v_rd_;
	std::vector<double> v_td_;

	std::vector<std::vector<double> > v_rot_cov_;

	std::vector<bool> cov_rot_state_;
	bool full_cov_rot_state_;

	std::vector<std::vector<Pose> > v_pose_;
	// v_pose_[idx_ref][indices_[idx_data][i]], v_pose_[idx_data][indices_[idx_data][i]] as the screw motion pair
	std::vector<std::vector<int> > indices_;


	size_t frame_cnt_, pose_cnt_;
};
