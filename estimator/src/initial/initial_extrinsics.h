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

#pragma once

#include <vector>
#include <queue>
#include <map>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <ros/console.h>

#include "common/types/type.h"
#include "common/algos/math.hpp"

#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

// maintain a priority_queue
// the left (first) should have larger w (higher_priority) than the right (second)
struct rotCmp
{
	bool operator()(const std::pair<size_t, std::vector<Pose> > &pose_r, const std::pair<size_t, std::vector<Pose> > &pose_l)
	{
		return (pose_l.second[0].q_.w() > pose_r.second[0].q_.w()); 
	}
};

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialExtrinsics
{
public:
	InitialExtrinsics();
	void clearState();
	void setParameter();

	bool addPose(const std::vector<Pose> &pose_laser);

	bool calibExRotation(const size_t &idx_ref, const size_t &idx_data, Pose &calib_result);
	bool calibExTranslation(const size_t &idx_ref, const size_t &idx_data, Pose &calib_result);
	bool calibExTranslationPlanar(const size_t &idx_ref, const size_t &idx_data);
	bool calibExTranslationNonPlanar(const size_t &idx_ref, const size_t &idx_data);
	void calibTimeDelay(const size_t &idx_ref, const size_t &idx_data);

	bool setCovRotation(const size_t &idx);
	bool setCovTranslation(const size_t &idx);

	bool checkScrewMotion(const Pose &pose_ref, const Pose &pose_data);
	void saveStatistics();

	void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2, cv::Mat_<double> &t1, cv::Mat_<double> &t2);

	std::vector<Pose> calib_ext_;

	std::vector<double> v_rd_;
	std::vector<double> v_td_;

	std::vector<std::vector<double> > v_rot_cov_, v_pos_cov_;
	std::vector<bool> cov_rot_state_, cov_pos_state_;
	bool full_cov_rot_state_, full_cov_pos_state_;
	double rot_cov_thre_;

	std::priority_queue<std::pair<size_t, std::vector<Pose> >, 
						std::vector<std::pair<size_t, std::vector<Pose> > >, rotCmp> pq_pose_;
	std::vector<std::vector<Pose> > v_pose_;

						
	// v_pose_[idx_ref][indices_[idx_data][i]], v_pose_[idx_data][indices_[idx_data][i]] as the screw motion pair
	std::vector<std::vector<int> > indices_;

	size_t frame_cnt_, pose_cnt_;

	std::vector<Eigen::MatrixXd> Q_;

	std::pair<size_t, std::vector<Pose> > pose_laser_add_;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
