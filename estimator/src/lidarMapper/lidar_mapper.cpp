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

#define PCL_NO_PRECOMPILE

#include "lidar_mapper.h"

using namespace common;

SaveStatistics save_statistics;

const float CUBE_SIZE = 50.0;
const float CUBE_HALF = CUBE_SIZE / 2;

int frame_cnt = 0;

double time_laser_cloud_surf_last = 0;
double time_laser_cloud_corner_last = 0;
double time_laser_cloud_full_res = 0;
double time_laser_odometry = 0;
double time_ext = 0;

// the center position of cude for mapping
int laser_cloud_cen_width = 10; // x
int laser_cloud_cen_height = 10; // y
int laser_cloud_cen_depth = 5; // z

// the cude size for mapping
const int laser_cloud_width = 21;
const int laser_cloud_height = 21;
const int laser_cloud_depth = 11;
const int laser_cloud_num = laser_cloud_width * laser_cloud_height * laser_cloud_depth; //4851

int laser_cloud_valid_ind[125];
int laser_cloud_surrond_ind[125];

// input: from odom
PointICloud::Ptr laser_cloud_surf_last(new PointICloud());
PointICloud::Ptr laser_cloud_corner_last(new PointICloud());

// surround points in map to build tree
PointICloud::Ptr laser_cloud_surf_from_map(new PointICloud());
PointICovCloud::Ptr laser_cloud_surf_from_map_cov(new PointICovCloud());

PointICloud::Ptr laser_cloud_corner_from_map(new PointICloud());
PointICovCloud::Ptr laser_cloud_corner_from_map_cov(new PointICovCloud());

//input & output: points in one frame. local --> global
PointICloud::Ptr laser_cloud_full_res(new PointICloud());

// points in every cube
PointICovCloud::Ptr laser_cloud_surf_array_cov[laser_cloud_num];
PointICovCloud::Ptr laser_cloud_corner_array_cov[laser_cloud_num];

//kd-tree
pcl::KdTreeFLANN<PointIWithCov>::Ptr kdtree_surf_from_map(new pcl::KdTreeFLANN<PointIWithCov>());
pcl::KdTreeFLANN<PointIWithCov>::Ptr kdtree_corner_from_map(new pcl::KdTreeFLANN<PointIWithCov>());

// wmap_T_curr = wmap_T_odom * wodom_T_curr;
// transformation between odom's world and map's world frame
double para_pose[SIZE_POSE];
Pose pose_wmap_prev, pose_wmap_curr, pose_wmap_wodom, pose_wodom_curr;
std::vector<std::pair<double, Pose> > pose_keyframe;

bool new_keyframe;

// downsampling voxel grid
pcl::VoxelGridCovarianceMLOAM<PointI> down_size_filter_surf;
pcl::VoxelGridCovarianceMLOAM<PointIWithCov> down_size_filter_surf_map_cov;
pcl::VoxelGridCovarianceMLOAM<PointI> down_size_filter_corner;
pcl::VoxelGridCovarianceMLOAM<PointIWithCov> down_size_filter_corner_map_cov;

std::vector<int> point_search_ind;
std::vector<float> point_search_sq_dis;

nav_msgs::Path laser_after_mapped_path;

ros::Publisher pub_laser_cloud_surround, pub_laser_cloud_map;
ros::Publisher pub_laser_cloud_full_res;
ros::Publisher pub_laser_cloud_surf_last_res, pub_laser_cloud_corner_last_res;
ros::Publisher pub_odom_aft_mapped, pub_odom_aft_mapped_high_frec, pub_laser_after_mapped_path;

// extrinsics
mloam_msgs::Extrinsics extrinsics;
std::vector<Eigen::Matrix3d> r_ext;
std::vector<Eigen::Vector3d> t_ext;
std::vector<Pose> pose_ext;

std::vector<PointICovCloud> laser_cloud_surf_split_cov;
std::vector<PointICovCloud> laser_cloud_corner_split_cov;

// thread data buffer
std::queue<sensor_msgs::PointCloud2ConstPtr> surf_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> corner_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> full_res_buf;
std::queue<nav_msgs::Odometry::ConstPtr> odometry_buf;
std::queue<mloam_msgs::ExtrinsicsConstPtr> ext_buf;
std::mutex m_buf;

FeatureExtract f_extract;

int UNCER_AWARE_ON = 1;
std::vector<Eigen::Matrix<double, 6, 6> > cov_ext;
Eigen::Matrix<double, 6, 6> cov_mapping;

std::vector<Eigen::Matrix<double, 1, 6> > d_factor_list;
std::vector<Eigen::Matrix<double, 6, 6> > d_eigvec_list;

std::vector<Pose> pose_compound;
std::vector<Eigen::Matrix<double, 6, 6> > cov_compound;

std::vector<double> cov_mapping_list;

double total_mapping = 0.0;

pcl::PCDWriter pcd_writer;

Eigen::Matrix<double, 6, 6> mat_P;
bool is_degenerate;

int toCubeIndex(const int &i, const int &j, const int &k)
{
	return (i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k);
}

// set current pose after odom
void transformAssociateToMap()
{
	// q_w_curr = q_wmap_wodom * q_wodom_curr;
	// t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
	pose_wmap_curr = pose_wmap_wodom * pose_wodom_curr;
	// std::cout << "pose_wmap_curr: " << pose_wmap_curr << std::endl;
}

// update the transformation between map's world to odom's world after map
void transformUpdate()
{
	// q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	// t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
	pose_wmap_wodom = pose_wmap_curr * pose_wodom_curr.inverse();
	// std::cout << "pose_wmap_wodom: " << pose_wmap_wodom << std::endl;
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_surf_last)
{
	m_buf.lock();
	surf_last_buf.push(laser_cloud_surf_last);
	m_buf.unlock();
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_corner_last)
{
	m_buf.lock();
	corner_last_buf.push(laser_cloud_corner_last);
	m_buf.unlock();
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_full_res)
{
	m_buf.lock();
	full_res_buf.push(laser_cloud_full_res);
	m_buf.unlock();
}

void extrinsicsHandler(const mloam_msgs::ExtrinsicsConstPtr &ext)
{
	m_buf.lock();
	ext_buf.push(ext);
	m_buf.unlock();
}

//receive odomtry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laser_odom)
{
	m_buf.lock();
	odometry_buf.push(laser_odom);
	m_buf.unlock();

	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = laser_odom->pose.pose.orientation.x;
	q_wodom_curr.y() = laser_odom->pose.pose.orientation.y;
	q_wodom_curr.z() = laser_odom->pose.pose.orientation.z;
	q_wodom_curr.w() = laser_odom->pose.pose.orientation.w;
	t_wodom_curr.x() = laser_odom->pose.pose.position.x;
	t_wodom_curr.y() = laser_odom->pose.pose.position.y;
	t_wodom_curr.z() = laser_odom->pose.pose.position.z;

	Pose pose_wmap_curr_ini = pose_wmap_wodom * pose_wodom_curr;
	nav_msgs::Odometry odom_aft_mapped;
	odom_aft_mapped.header.frame_id = "/world";
	odom_aft_mapped.child_frame_id = "/aft_mapped";
	odom_aft_mapped.header.stamp = laser_odom->header.stamp;
	odom_aft_mapped.pose.pose.orientation.x = pose_wmap_curr_ini.q_.x();
	odom_aft_mapped.pose.pose.orientation.y = pose_wmap_curr_ini.q_.y();
	odom_aft_mapped.pose.pose.orientation.z = pose_wmap_curr_ini.q_.z();
	odom_aft_mapped.pose.pose.orientation.w = pose_wmap_curr_ini.q_.w();
	odom_aft_mapped.pose.pose.position.x = pose_wmap_curr_ini.t_.x();
	odom_aft_mapped.pose.pose.position.y = pose_wmap_curr_ini.t_.y();
	odom_aft_mapped.pose.pose.position.z = pose_wmap_curr_ini.t_.z();
	pub_odom_aft_mapped_high_frec.publish(odom_aft_mapped); // publish (k-1)th oldest map * kth newest odom
}

void vector2Double()
{
	para_pose[0] = pose_wmap_curr.t_(0);
	para_pose[1] = pose_wmap_curr.t_(1);
	para_pose[2] = pose_wmap_curr.t_(2);
	para_pose[3] = pose_wmap_curr.q_.x();
	para_pose[4] = pose_wmap_curr.q_.y();
	para_pose[5] = pose_wmap_curr.q_.z();
	para_pose[6] = pose_wmap_curr.q_.w();
}

void double2Vector()
{
	pose_wmap_curr.t_ = Eigen::Vector3d(para_pose[0], para_pose[1], para_pose[2]);
	pose_wmap_curr.q_ = Eigen::Quaterniond(para_pose[6], para_pose[3], para_pose[4], para_pose[5]);
}

void insertNewKeyframe()
{
	new_keyframe = true;
	if (common::sqrSum(pose_wmap_curr.t_[0] - pose_wmap_prev.t_[0],
					   pose_wmap_curr.t_[1] - pose_wmap_prev.t_[1],
					   pose_wmap_curr.t_[2] - pose_wmap_prev.t_[2]) < 0.3)
	{
		new_keyframe = false;
	}
	if ((!new_keyframe) && (pose_keyframe.size() != 0)) return;
	pose_wmap_prev = pose_wmap_curr;
	
	// if (pose_keyframe.size() == 0)
	// {
	// 	pose_keyframe.push_back(std::make_pair(time_laser_cloud_surf_last, pose_wmap_curr));
	// } else
	// {
	// 	pose_keyframe.push_back(std::make_pair(time_laser_cloud_surf_last, pose_wmap_curr));
	// 	// construct a pose graph
	// }
	pose_keyframe.push_back(std::make_pair(time_laser_cloud_surf_last, pose_wmap_curr));
}

void process()
{
	while (1)
	{
		if (!ros::ok()) break;
		while (!surf_last_buf.empty() && !corner_last_buf.empty() &&
			   !full_res_buf.empty() && !ext_buf.empty() && !odometry_buf.empty())
		{
			//***************************************************************************
			// step 1: pop up subscribed data
			m_buf.lock();
			while (!corner_last_buf.empty() && corner_last_buf.front()->header.stamp.toSec() < surf_last_buf.front()->header.stamp.toSec())
				corner_last_buf.pop();
			if (corner_last_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			while (!full_res_buf.empty() && full_res_buf.front()->header.stamp.toSec() < surf_last_buf.front()->header.stamp.toSec())
				full_res_buf.pop();
			if (full_res_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			while (!odometry_buf.empty() && odometry_buf.front()->header.stamp.toSec() < surf_last_buf.front()->header.stamp.toSec())
				odometry_buf.pop();
			if (odometry_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			while (!ext_buf.empty() && ext_buf.front()->header.stamp.toSec() < surf_last_buf.front()->header.stamp.toSec())
				ext_buf.pop();
			if (ext_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			time_laser_cloud_surf_last = surf_last_buf.front()->header.stamp.toSec();
			time_laser_cloud_corner_last = corner_last_buf.front()->header.stamp.toSec();
			time_laser_cloud_full_res = full_res_buf.front()->header.stamp.toSec();
			time_laser_odometry = odometry_buf.front()->header.stamp.toSec();
			time_ext = ext_buf.front()->header.stamp.toSec();

			if (time_laser_cloud_surf_last != time_laser_cloud_corner_last ||
				time_laser_cloud_surf_last != time_laser_cloud_full_res ||
				time_laser_cloud_surf_last != time_laser_odometry ||
				time_laser_cloud_surf_last != time_ext)
			{
				printf("time surf: %f, corner: %f, full: %f, odom: %f\n, ext: %f\n",
					   time_laser_cloud_surf_last, time_laser_cloud_corner_last, 
					   time_laser_cloud_full_res, time_laser_odometry, time_ext);
				printf("unsync messeage!");
				m_buf.unlock();
				break;
			}

			laser_cloud_surf_last->clear();
			pcl::fromROSMsg(*surf_last_buf.front(), *laser_cloud_surf_last);
			// roiCloudFilter(*laser_cloud_surf_last, ROI_RANGE_MAPPING);
			surf_last_buf.pop();

			laser_cloud_corner_last->clear();
			pcl::fromROSMsg(*corner_last_buf.front(), *laser_cloud_corner_last);
			// roiCloudFilter(*laser_cloud_corner_last, ROI_RANGE_MAPPING);
			corner_last_buf.pop();

			laser_cloud_full_res->clear();
			pcl::fromROSMsg(*full_res_buf.front(), *laser_cloud_full_res);
			full_res_buf.pop();
			// printf("input full:%d, surf:%d\n", laser_cloud_full_res->size(), laser_cloud_surf_last->size());

			pose_wodom_curr.q_ = Eigen::Quaterniond(odometry_buf.front()->pose.pose.orientation.w,
													odometry_buf.front()->pose.pose.orientation.x,
													odometry_buf.front()->pose.pose.orientation.y,
													odometry_buf.front()->pose.pose.orientation.z);
			pose_wodom_curr.t_ = Eigen::Vector3d(odometry_buf.front()->pose.pose.position.x,
												 odometry_buf.front()->pose.pose.position.y,
												 odometry_buf.front()->pose.pose.position.z);
			odometry_buf.pop();

			extrinsics = *ext_buf.front();
			if (!extrinsics.status)
			{
				std::cout << common::YELLOW << "Accurate extrinsic calibration!" << common::RESET << std::endl;
				for (size_t n = 0; n < NUM_OF_LASER; n++)
				{
					pose_ext[n].q_ = Eigen::Quaterniond(extrinsics.odoms[n].pose.pose.orientation.w,
														extrinsics.odoms[n].pose.pose.orientation.x,
														extrinsics.odoms[n].pose.pose.orientation.y,
														extrinsics.odoms[n].pose.pose.orientation.z);
					pose_ext[n].t_ = Eigen::Vector3d(extrinsics.odoms[n].pose.pose.position.x,
													 extrinsics.odoms[n].pose.pose.position.y,
													 extrinsics.odoms[n].pose.pose.position.z);
					for (size_t i = 0; i < 6; i++)
						for (size_t j = 0; j < 6; j++)
							cov_ext[n](i, j) = double(extrinsics.odoms[n].pose.covariance[i * 6 + j]);
				}
			}
			ext_buf.pop();

			while (!surf_last_buf.empty())
			{
				surf_last_buf.pop();
				std::cout << common::GREEN << "drop lidar frame in mapping for real time performance" << common::RESET << std::endl;
			}
			
			if (extrinsics.status) continue;
			m_buf.unlock();

			frame_cnt++;
			TicToc t_whole_mapping;
			transformAssociateToMap();

// **********************************************************************
// step 2: move current map to the managed cube area
			TicToc t_shift;
			int center_cub_i = int((pose_wmap_curr.t_.x() + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_width; // the cube id
			int center_cub_j = int((pose_wmap_curr.t_.y() + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_height;
			int center_cub_k = int((pose_wmap_curr.t_.z() + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_depth;
			if (pose_wmap_curr.t_.x() + CUBE_HALF < 0) center_cub_i--;
			if (pose_wmap_curr.t_.y() + CUBE_HALF < 0) center_cub_j--;
			if (pose_wmap_curr.t_.z() + CUBE_HALF < 0) center_cub_k--;
			// printf("size_cube: %d, %d, %d\n", laser_cloud_width, laser_cloud_height, laser_cloud_depth);
			// printf("center_cube: %d, %d, %d\n", center_cub_i, center_cub_j, center_cub_k);

			// 3 < center_cub_i < 18， 3 < center_cub_j < 18, 3 < center_cub_k < 8
			// laser_cloud_num = laser_cloud_width * laser_cloud_height * laser_cloud_depth; 21*21*11=4851
			// indicate the map in the -, so the sweep the order of pointer
			while (center_cub_i < 3)
			{
				for (int j = 0; j < laser_cloud_height; j++)
				{
					for (int k = 0; k < laser_cloud_depth; k++)
					{
						for (int i = laser_cloud_width - 1; i >= 1; i--)
						{
							int old_cube_idx = toCubeIndex(i, j, k);
							int new_cube_idx = toCubeIndex(i - 1, j, k);
							std::swap(laser_cloud_surf_array_cov[old_cube_idx], laser_cloud_surf_array_cov[new_cube_idx]);
							std::swap(laser_cloud_corner_array_cov[old_cube_idx], laser_cloud_corner_array_cov[new_cube_idx]);
						}
					}
				}
				center_cub_i++;
				laser_cloud_cen_width++;
			}

			while (center_cub_i >= laser_cloud_width - 3)
			{
				for (int j = 0; j < laser_cloud_height; j++)
				{
					for (int k = 0; k < laser_cloud_depth; k++)
					{
						for (int i = 0; i < laser_cloud_width - 1; i++)
						{
							int old_cube_idx = toCubeIndex(i, j, k);
							int new_cube_idx = toCubeIndex(i + 1, j, k);
							std::swap(laser_cloud_surf_array_cov[old_cube_idx], laser_cloud_surf_array_cov[new_cube_idx]);
							std::swap(laser_cloud_corner_array_cov[old_cube_idx], laser_cloud_corner_array_cov[new_cube_idx]);
						}
					}
				}
				center_cub_i--;
				laser_cloud_cen_width--;
			}

			while (center_cub_j < 3)
			{
				for (int i = 0; i < laser_cloud_width; i++)
				{
					for (int k = 0; k < laser_cloud_depth; k++)
					{
						for (int j = laser_cloud_height - 1; j >= 1; j--)
						{
							int old_cube_idx = toCubeIndex(i, j, k);
							int new_cube_idx = toCubeIndex(i, j - 1, k);
							std::swap(laser_cloud_surf_array_cov[old_cube_idx], laser_cloud_surf_array_cov[new_cube_idx]);
							std::swap(laser_cloud_corner_array_cov[old_cube_idx], laser_cloud_corner_array_cov[new_cube_idx]);
						}
					}
				}
				center_cub_j++;
				laser_cloud_cen_height++;
			}

			while (center_cub_j >= laser_cloud_height - 3)
			{
				for (int i = 0; i < laser_cloud_width; i++)
				{
					for (int k = 0; k < laser_cloud_depth; k++)
					{
						for (int j = 0; j < laser_cloud_height - 1; j++)
						{
							int old_cube_idx = toCubeIndex(i, j, k);
							int new_cube_idx = toCubeIndex(i, j + 1, k);
							std::swap(laser_cloud_surf_array_cov[old_cube_idx], laser_cloud_surf_array_cov[new_cube_idx]);
							std::swap(laser_cloud_corner_array_cov[old_cube_idx], laser_cloud_corner_array_cov[new_cube_idx]);
						}
					}
				}
				center_cub_j--;
				laser_cloud_cen_height--;
			}

			while (center_cub_k < 3)
			{
				for (int i = 0; i < laser_cloud_width; i++)
				{
					for (int j = 0; j < laser_cloud_height; j++)
					{
						for (int k = laser_cloud_depth - 1; k >= 1; k--)
						{
							int old_cube_idx = toCubeIndex(i, j, k);
							int new_cube_idx = toCubeIndex(i, j, k - 1);
							std::swap(laser_cloud_surf_array_cov[old_cube_idx], laser_cloud_surf_array_cov[new_cube_idx]);
							std::swap(laser_cloud_corner_array_cov[old_cube_idx], laser_cloud_corner_array_cov[new_cube_idx]);
						}
					}
				}
				center_cub_k++;
				laser_cloud_cen_depth++;
			}

			while (center_cub_k >= laser_cloud_depth - 3)
			{
				for (int i = 0; i < laser_cloud_width; i++)
				{
					for (int j = 0; j < laser_cloud_height; j++)
					{
						for (int k = 0; k < laser_cloud_depth - 1; k++)
						{
							int old_cube_idx = toCubeIndex(i, j, k);
							int new_cube_idx = toCubeIndex(i, j, k + 1);
							std::swap(laser_cloud_surf_array_cov[old_cube_idx], laser_cloud_surf_array_cov[new_cube_idx]);
							std::swap(laser_cloud_corner_array_cov[old_cube_idx], laser_cloud_corner_array_cov[new_cube_idx]);
						}
					}
				}
				center_cub_k--;
				laser_cloud_cen_depth--;
			}

			// select the nearest 5*5*3=125 visiable cubes as the candidate cubes
			int laser_cloud_valid_num = 0; // save the valid cube number
			int laser_cloud_surround_num = 0; // save the surround cube number
			for (int i = center_cub_i - 2; i <= center_cub_i + 2; i++)
			{
				for (int j = center_cub_j - 2; j <= center_cub_j + 2; j++)
				{
					for (int k = center_cub_k - 1; k <= center_cub_k + 1; k++)
					{
						if (i >= 0 && i < laser_cloud_width &&
							j >= 0 && j < laser_cloud_height &&
							k >= 0 && k < laser_cloud_depth)
						{
							int cur_cube_idx = toCubeIndex(i, j, k);
							laser_cloud_valid_ind[laser_cloud_valid_num] = cur_cube_idx;
							laser_cloud_valid_num++;
							laser_cloud_surrond_ind[laser_cloud_surround_num] = cur_cube_idx;
							laser_cloud_surround_num++;
						}
					}
				}
			}

			// load surf map features
			laser_cloud_surf_from_map_cov->clear();
			laser_cloud_corner_from_map_cov->clear();
			for (int i = 0; i < laser_cloud_valid_num; i++)
			{
				*laser_cloud_surf_from_map_cov += *laser_cloud_surf_array_cov[laser_cloud_valid_ind[i]];
				*laser_cloud_corner_from_map_cov += *laser_cloud_corner_array_cov[laser_cloud_valid_ind[i]];
			}
			// printf("map prepare time: %fms\n", t_shift.toc());

			// filter

// **********************************************************************
// step 3: process current input
			PointICloud::Ptr laser_cloud_surf_stack(new PointICloud());
			down_size_filter_surf.setInputCloud(laser_cloud_surf_last);
			down_size_filter_surf.filter(*laser_cloud_surf_stack);
			PointICloud::Ptr laser_cloud_corner_stack(new PointICloud());
			down_size_filter_corner.setInputCloud(laser_cloud_corner_last);
			down_size_filter_corner.filter(*laser_cloud_corner_stack);
			LOG_EVERY_N(INFO, 10) << "input surf num: " << laser_cloud_surf_stack->size() 
			 					   << " corner num: " << laser_cloud_corner_stack->size();

			//**************************************************************
			for (size_t n = 0; n < NUM_OF_LASER; n++)
			{
				laser_cloud_surf_split_cov[n].clear();
				laser_cloud_corner_split_cov[n].clear();
			}
			// propagate the extrinsic uncertainty on points
			for (PointI &point_ori: *laser_cloud_surf_stack)
			{
				int idx = int(point_ori.intensity); // indicate the lidar id
				PointI point_sel;
				Eigen::Matrix3d cov_point = Eigen::Matrix3d::Zero();
				pointAssociateToMap(point_ori, point_sel, pose_ext[idx].inverse());
				evalPointUncertainty(point_sel, cov_point, pose_ext[idx], cov_ext[idx]);
				if (!UNCER_AWARE_ON) cov_point = COV_MEASUREMENT; // add extrinsic perturbation
				if (cov_point.trace() <= TRACE_THRESHOLD_BEFORE_MAPPING)
				{
					PointIWithCov point_cov(point_ori, cov_point.cast<float>());
					laser_cloud_surf_split_cov[idx].push_back(point_cov);
				}
			}
			for (PointI &point_ori : *laser_cloud_corner_stack)
			{
				int idx = int(point_ori.intensity); // indicate the lidar id
				PointI point_sel;
				Eigen::Matrix3d cov_point = Eigen::Matrix3d::Zero();
				pointAssociateToMap(point_ori, point_sel, pose_ext[idx].inverse());
				evalPointUncertainty(point_sel, cov_point, pose_ext[idx], cov_ext[idx]);
				if (!UNCER_AWARE_ON) cov_point = COV_MEASUREMENT; // add extrinsic perturbation
				if (cov_point.trace() <= TRACE_THRESHOLD_BEFORE_MAPPING)
				{
					PointIWithCov point_cov(point_ori, cov_point.cast<float>());
					laser_cloud_corner_split_cov[idx].push_back(point_cov);
				}
			}

//***************************************************************************
// step 4: perform scan-to-map optimization
			size_t laser_cloud_surf_from_map_num = laser_cloud_surf_from_map_cov->points.size();
			size_t laser_cloud_corner_from_map_num = laser_cloud_corner_from_map_cov->points.size();
			printf("map surf num: %lu, corner num: %lu\n", laser_cloud_surf_from_map_num, laser_cloud_corner_from_map_num);
			if ((laser_cloud_surf_from_map_num > 100) && (laser_cloud_corner_from_map_num > 10))
			{
				TicToc t_opt, t_tree;
				kdtree_surf_from_map->setInputCloud(laser_cloud_surf_from_map_cov);
				kdtree_corner_from_map->setInputCloud(laser_cloud_corner_from_map_cov);
				printf("build tree time %fms\n", t_tree.toc());
				printf("********************************\n");
				for (int iter_cnt = 0; iter_cnt < 2; iter_cnt++)
				{
					ceres::Problem problem;
					ceres::Solver::Summary summary;
					ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

					ceres::Solver::Options options;
					options.linear_solver_type = ceres::DENSE_SCHUR;
					options.max_num_iterations = 30;
					// options.max_solver_time_in_seconds = 0.04;
					// options.num_threads = 3;
					options.minimizer_progress_to_stdout = false;
					options.check_gradients = false;
					options.gradient_check_relative_precision = 1e-4;

					vector2Double();
					
					std::vector<double *> para_ids;
					std::vector<ceres::internal::ResidualBlock *> res_ids_proj;
					PoseLocalParameterization *local_parameterization = new PoseLocalParameterization();
					local_parameterization->setParameter();
					problem.AddParameterBlock(para_pose, SIZE_POSE, local_parameterization);
					para_ids.push_back(para_pose);

					// ******************************************************
					TicToc t_match_features;
					std::vector<PointPlaneFeature> map_features;
					size_t surf_num = 0, corner_num = 0;
					for (size_t n = 0; n < NUM_OF_LASER; n++)
					{
						int n_neigh = 5;
						if (POINT_PLANE_FACTOR)
						{
							std::vector<PointPlaneFeature> feature_frame;
							f_extract.matchSurfFromMap(kdtree_surf_from_map,
													   *laser_cloud_surf_from_map_cov,
													   laser_cloud_surf_split_cov[n],
													   pose_wmap_curr,
													   feature_frame,
													   n,
													   n_neigh,
													   true);
							map_features.insert(map_features.end(), feature_frame.begin(), feature_frame.end());
							surf_num += feature_frame.size();
						}
						if (POINT_EDGE_FACTOR)
						{
							std::vector<PointPlaneFeature> feature_frame;
							f_extract.matchCornerFromMap(kdtree_corner_from_map,
														 *laser_cloud_corner_from_map_cov,
														 laser_cloud_corner_split_cov[n],
														 pose_wmap_curr,
														 feature_frame,
														 n,
														 n_neigh,
														 true);
							map_features.insert(map_features.end(), feature_frame.begin(), feature_frame.end());
							corner_num += feature_frame.size();
						}
					}
					printf("matching features time: %fms\n", t_match_features.toc());
					LOG_EVERY_N(INFO, 10) << "matching surf & corner features num: " <<  surf_num << ", " << corner_num;

					std::vector<size_t> sel_feature_idx;
					goodFeatureSelect(para_pose, 
									  laser_cloud_surf_split_cov, laser_cloud_corner_split_cov,
									  map_features, map_features.size(), 
									  sel_feature_idx, FLAGS_gf_ratio);
					printf("selected features num: %lu(%lu)\n", sel_feature_idx.size(), surf_num + corner_num);

					if ((frame_cnt % 100 == 0) && (FLAGS_gf_ratio != 1.0))
						writeFeature(sel_feature_idx, map_features);

					TicToc t_add_constraints;
					CHECK_JACOBIAN = 0;
					for (size_t fid : sel_feature_idx)
					{
						const PointPlaneFeature &feature = map_features[fid];
						Eigen::Matrix3d cov_matrix = Eigen::Matrix3d::Identity();
						if (feature.type_ == 's')
							extractCov(laser_cloud_surf_split_cov[feature.laser_idx_].points[feature.idx_], cov_matrix);
						else if (feature.type_ == 'c')
							extractCov(laser_cloud_corner_split_cov[feature.laser_idx_].points[feature.idx_], cov_matrix);
						LidarMapPlaneNormFactor *f = new LidarMapPlaneNormFactor(feature.point_, feature.coeffs_, cov_matrix);
						ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function, para_pose);
						res_ids_proj.push_back(res_id);
						if (CHECK_JACOBIAN)
						{
							const double **tmp_param = new const double *[1];
							tmp_param[0] = para_pose;
							f->check(tmp_param);
							CHECK_JACOBIAN = 0;
						}
					}
					printf("add constraints: %fms\n", t_add_constraints.toc());
					// ******************************************************

					if (iter_cnt == 0)
					{
						TicToc t_eval_H;
						double cost = 0.0;
						ceres::CRSMatrix jaco;
						ceres::Problem::EvaluateOptions e_option;
						e_option.parameter_blocks = para_ids;
						e_option.residual_blocks = res_ids_proj;
						problem.Evaluate(e_option, &cost, nullptr, nullptr, &jaco);

						Eigen::Matrix<double, 6, 6> mat_H;
						evalHessian(jaco, mat_H);
						evalDegenracy(mat_H, local_parameterization);
						cov_mapping = mat_H.inverse(); // covariance of sensor noise: A New Approach to 3D ICP Covariance Estimation/ Censi's approach
						cov_mapping_list.push_back(cov_mapping.trace());
						is_degenerate = local_parameterization->is_degenerate_;
						LOG_EVERY_N(INFO, 10) << "logdet of H: " << common::logDet(mat_H * 134, true);
						LOG_EVERY_N(INFO, 10) << "pose covariance trace: " << cov_mapping.trace();
						printf("evaluate H: %fms\n", t_eval_H.toc());
					} 
					else if (is_degenerate)
					{
						local_parameterization->V_update_ = mat_P;
					}				
					// *********************************************************

					TicToc t_solver;
					ceres::Solve(options, &problem, &summary);
					std::cout << summary.BriefReport() << std::endl;
					// std::cout << summary.FullReport() << std::endl;
					printf("mapping solver time: %fms\n", t_solver.toc());		

					double2Vector();
					std::cout << iter_cnt << "th result: " << pose_wmap_curr << std::endl;
					if (iter_cnt == 0) printf("-------------------------------------\n");
				}
				printf("********************************\n");
				printf("mapping optimization time: %fms\n", t_opt.toc());
			}
			else
			{
				ROS_WARN("Map surf num is not enough");
			}
			transformUpdate();

			// *******************************************************************
			// check distance and insert new keyframes
			insertNewKeyframe();

// *******************************************************************
// step 5: add newest surf points to the map according to a new keyframe
			if (new_keyframe)
			{
				TicToc t_add;
				for (size_t n = 0; n < NUM_OF_LASER; n++)
				{
					compoundPoseWithCov(pose_wmap_curr, cov_mapping, pose_ext[n], cov_ext[n], pose_compound[n], cov_compound[n], 2);
					// move the surf points from the lastest frame to different cubes
					for (const PointIWithCov &point_ori : laser_cloud_surf_split_cov[n])
					{
						PointIWithCov point_sel, point_cov;
						Eigen::Matrix3d cov_point = Eigen::Matrix3d::Zero();
						pointAssociateToMap(point_ori, point_sel, pose_ext[n].inverse());
						evalPointUncertainty(point_sel, cov_point, pose_compound[n], cov_compound[n]);
						// 0.01 for RHD02lab
						// if (!UNCER_AWARE_ON || cov_mapping.trace() < 0.03) cov_point = COV_MEASUREMENT; // add pose and extrinsic perturbation 
						if (!UNCER_AWARE_ON) cov_point = COV_MEASUREMENT; // add pose and extrinsic perturbation 
						if (cov_point.trace() > TRACE_THRESHOLD_AFTER_MAPPING) continue;
						pointAssociateToMap(point_ori, point_cov, pose_wmap_curr);
						updateCov(point_cov, cov_point);

						int cube_i = int((point_cov.x + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_width;
						int cube_j = int((point_cov.y + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_height;
						int cube_k = int((point_cov.z + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_depth;

						if (point_cov.x + CUBE_HALF < 0) cube_i--;
						if (point_cov.y + CUBE_HALF < 0) cube_j--;
						if (point_cov.z + CUBE_HALF < 0) cube_k--;

						if (cube_i >= 0 && cube_i < laser_cloud_width &&
							cube_j >= 0 && cube_j < laser_cloud_height &&
							cube_k >= 0 && cube_k < laser_cloud_depth)
						{
							int cur_cube_idx = toCubeIndex(cube_i, cube_j, cube_k);
							laser_cloud_surf_array_cov[cur_cube_idx]->push_back(point_cov);
						}
					}
					for (const PointIWithCov &point_ori : laser_cloud_corner_split_cov[n])
					{
						PointIWithCov point_sel, point_cov;
						Eigen::Matrix3d cov_point = Eigen::Matrix3d::Zero();
						pointAssociateToMap(point_ori, point_sel, pose_ext[n].inverse());
						evalPointUncertainty(point_sel, cov_point, pose_compound[n], cov_compound[n]);
						if (!UNCER_AWARE_ON) cov_point = COV_MEASUREMENT; // add pose and extrinsic perturbation
						if (cov_point.trace() > TRACE_THRESHOLD_AFTER_MAPPING) continue;
						pointAssociateToMap(point_ori, point_cov, pose_wmap_curr);
						updateCov(point_cov, cov_point);

						int cube_i = int((point_cov.x + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_width;
						int cube_j = int((point_cov.y + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_height;
						int cube_k = int((point_cov.z + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_depth;

						if (point_cov.x + CUBE_HALF < 0)
							cube_i--;
						if (point_cov.y + CUBE_HALF < 0)
							cube_j--;
						if (point_cov.z + CUBE_HALF < 0)
							cube_k--;

						if (cube_i >= 0 && cube_i < laser_cloud_width &&
							cube_j >= 0 && cube_j < laser_cloud_height &&
							cube_k >= 0 && cube_k < laser_cloud_depth)
						{
							int cur_cube_idx = toCubeIndex(cube_i, cube_j, cube_k);
							laser_cloud_corner_array_cov[cur_cube_idx]->push_back(point_cov);
						}
					}
				}
				printf("add points time: %fms\n", t_add.toc());

				// downsample the map (all map points including optimization or not optimization)
				TicToc t_filter;
				for (int i = 0; i < laser_cloud_valid_num; i++)
				{
					int ind = laser_cloud_valid_ind[i];

					PointICovCloud::Ptr tmp_surf(new PointICovCloud());
					down_size_filter_surf_map_cov.setInputCloud(laser_cloud_surf_array_cov[ind]);
					down_size_filter_surf_map_cov.filter(*tmp_surf);
					laser_cloud_surf_array_cov[ind] = tmp_surf;

					PointICovCloud::Ptr tmp_corner(new PointICovCloud());
					down_size_filter_corner_map_cov.setInputCloud(laser_cloud_corner_array_cov[ind]);
					down_size_filter_corner_map_cov.filter(*tmp_corner);
					laser_cloud_corner_array_cov[ind] = tmp_corner;
				}
				printf("filter time: %fms\n", t_filter.toc());

				// ************************************************************** publish feature and map data
				// publish surround map (use for optimization) for every 50 frame
				TicToc t_pub;
				if (pub_laser_cloud_surround.getNumSubscribers() != 0)
				{
					sensor_msgs::PointCloud2 laser_cloud_surround_msg;
					pcl::toROSMsg(*laser_cloud_surf_from_map_cov, laser_cloud_surround_msg);
					laser_cloud_surround_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
					laser_cloud_surround_msg.header.frame_id = "/world";
					pub_laser_cloud_surround.publish(laser_cloud_surround_msg);
					// printf("size of surround map: %d\n", laser_cloud_surrond.size());
				}

				if (pub_laser_cloud_map.getNumSubscribers() != 0)
				{
					PointICovCloud laser_cloud_map;
					for (int i = 0; i < laser_cloud_num; i++) laser_cloud_map += *laser_cloud_surf_array_cov[i];
					sensor_msgs::PointCloud2 laser_cloud_msg;
					pcl::toROSMsg(laser_cloud_map, laser_cloud_msg);
					laser_cloud_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
					laser_cloud_msg.header.frame_id = "/world";
					pub_laser_cloud_map.publish(laser_cloud_msg);
					// printf("size of cloud map: %d\n", laser_cloud_map.size());
				}
				LOG_EVERY_N(INFO, 1) << "mapping pub time: " << t_pub.toc() << "ms";
			}

			// publish registrated laser cloud
			for (PointI &point : *laser_cloud_full_res) pointAssociateToMap(point, point, pose_wmap_curr);
			sensor_msgs::PointCloud2 laser_cloud_full_res_msg;
			pcl::toROSMsg(*laser_cloud_full_res, laser_cloud_full_res_msg);
			laser_cloud_full_res_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
			laser_cloud_full_res_msg.header.frame_id = "/world";
			pub_laser_cloud_full_res.publish(laser_cloud_full_res_msg);

			for (PointI &point : *laser_cloud_surf_last) pointAssociateToMap(point, point, pose_wmap_curr);
			sensor_msgs::PointCloud2 laser_cloud_surf_last_msg;
			pcl::toROSMsg(*laser_cloud_surf_last, laser_cloud_surf_last_msg);
			laser_cloud_surf_last_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
			laser_cloud_surf_last_msg.header.frame_id = "/world";
			pub_laser_cloud_surf_last_res.publish(laser_cloud_surf_last_msg);

			for (PointI &point : *laser_cloud_corner_last) pointAssociateToMap(point, point, pose_wmap_curr);
			sensor_msgs::PointCloud2 laser_cloud_corner_last_msg;
			pcl::toROSMsg(*laser_cloud_corner_last, laser_cloud_corner_last_msg);
			laser_cloud_corner_last_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
			laser_cloud_corner_last_msg.header.frame_id = "/world";
			pub_laser_cloud_corner_last_res.publish(laser_cloud_corner_last_msg);

			std::cout << common::RED << "frame: " << frame_cnt
					  << ", whole mapping time " << t_whole_mapping.toc() << "ms" << common::RESET << std::endl;
			LOG_EVERY_N(INFO, 10) << "whole mapping time " << t_whole_mapping.toc() << "ms";
			total_mapping += t_whole_mapping.toc();

// **********************************************************************
// step 5: publish odom
			nav_msgs::Odometry odom_aft_mapped;
			odom_aft_mapped.header.frame_id = "/world";
			odom_aft_mapped.child_frame_id = "/aft_mapped";
			odom_aft_mapped.header.stamp = ros::Time().fromSec(time_laser_odometry);
			odom_aft_mapped.pose.pose.orientation.x = pose_wmap_curr.q_.x();
			odom_aft_mapped.pose.pose.orientation.y = pose_wmap_curr.q_.y();
			odom_aft_mapped.pose.pose.orientation.z = pose_wmap_curr.q_.z();
			odom_aft_mapped.pose.pose.orientation.w = pose_wmap_curr.q_.w();
			odom_aft_mapped.pose.pose.position.x = pose_wmap_curr.t_.x();
			odom_aft_mapped.pose.pose.position.y = pose_wmap_curr.t_.y();
			odom_aft_mapped.pose.pose.position.z = pose_wmap_curr.t_.z();
			for (size_t i = 0; i < 6; i++)
				for (size_t j = 0; j < 6; j++)
					odom_aft_mapped.pose.covariance[i * 6 + j] = float(cov_mapping(i, j));
			pub_odom_aft_mapped.publish(odom_aft_mapped);

			geometry_msgs::PoseStamped laser_after_mapped_pose;
			laser_after_mapped_pose.header = odom_aft_mapped.header;
			laser_after_mapped_pose.pose = odom_aft_mapped.pose.pose;
			laser_after_mapped_path.header.stamp = odom_aft_mapped.header.stamp;
			laser_after_mapped_path.header.frame_id = "/world";
			laser_after_mapped_path.poses.push_back(laser_after_mapped_pose);
			pub_laser_after_mapped_path.publish(laser_after_mapped_path);
			publishTF(odom_aft_mapped);

			// std::cout << "pose_wmap_curr: " << pose_wmap_curr << std::endl;
			printf("\n");
		}
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}
}

void evalHessian(const ceres::CRSMatrix &jaco, Eigen::Matrix<double, 6, 6> &mat_H)
{
	// printf("jacob: %d constraints, %d parameters\n", jaco.num_rows, jaco.num_cols); // 2000+, 6
	if (jaco.num_rows == 0) return;
	Eigen::SparseMatrix<double, Eigen::RowMajor> mat_J; // Jacobian is a diagonal matrix
	CRSMatrix2EigenMatrix(jaco, mat_J);
	Eigen::SparseMatrix<double, Eigen::RowMajor> mat_Jt = mat_J.transpose();
	Eigen::MatrixXd mat_JtJ = mat_Jt * mat_J;
	mat_H = mat_JtJ.block(0, 0, 6, 6) / 134;
}

void evalDegenracy(const Eigen::Matrix<double, 6, 6> &mat_H, PoseLocalParameterization *local_parameterization)
{
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > esolver(mat_H);
	Eigen::Matrix<double, 1, 6> mat_E = esolver.eigenvalues().real();	// 6*1
	Eigen::Matrix<double, 6, 6> mat_V_f = esolver.eigenvectors().real(); // 6*6, column is the corresponding eigenvector
	Eigen::Matrix<double, 6, 6> mat_V_p = mat_V_f;

	for (auto j = 0; j < mat_E.cols(); j++)
	{
		if (mat_E(0, j) < MAP_EIG_THRE)
		{
			mat_V_p.col(j) = Eigen::Matrix<double, 6, 1>::Zero();
			local_parameterization->is_degenerate_ = true;
		}
		else
		{
			break;
		}
	}
	d_factor_list.push_back(mat_E);
	d_eigvec_list.push_back(mat_V_f);
	LOG_EVERY_N(INFO, 10) << "D factor: " << mat_E(0, 0) << ", D vector: " << mat_V_f.col(0).transpose();
 	mat_P = (mat_V_f.transpose()).inverse() * mat_V_p.transpose(); // 6*6
	// std::cout << "jjiao:" << std::endl;
	// std::cout << "mat_E: " << mat_E << std::endl;
	// std::cout << "mat_V_f: " << std::endl << mat_V_f << std::endl;
	// std::cout << "mat_V_p: " << std::endl << mat_V_p << std::endl;
	// std::cout << "mat_P: " << std::endl << mat_P.transpose() << std::endl;

	if (local_parameterization->is_degenerate_)
	{
		local_parameterization->V_update_ = mat_P;
		// std::cout << "param " << i << " is degenerate !" << std::endl;
		// std::cout << mat_P.transpose() << std::endl;
	} else
	{
		is_degenerate = false;
	}
	

	// {
	// 	Eigen::Matrix<float, 6, 6> mat_H = mat_JtJ.cast<float>().block(0, 0, 6, 6) / 400.0;
	// 	cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
	// 	cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
	// 	cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
	// 	cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
	// 	cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
	//
	// 	cv::eigen2cv(mat_H, matAtA);
	// 	cv::eigen(matAtA, matE, matV);
	// 	matV.copyTo(matV2);
	// 	bool isDegenerate;
	// 	for (int i = 5; i >= 0; i--)
	// 	{
	// 		if (matE.at<float>(0, i) < 100.0)
	// 		{
	// 			for (int j = 0; j < 6; j++)
	// 			{
	// 				matV2.at<float>(i, j) = 0;
	// 			}
	// 			isDegenerate = true;
	// 		} else
	// 		{
	// 			break;
	// 		}
	// 	}
	// 	std::cout << "Zhang:" << std::endl;
	// 	std::cout << "mat_E: " << matE.t() << std::endl;
	// 	std::cout << "mat_V_f: " << std::endl << matV.t() << std::endl;
	// 	std::cout << "mat_V_p: " << std::endl << matV2.t() << std::endl;
	// 	matP = matV.inv() * matV2;
	// 	std::cout << "mat_P: " << std::endl << matP << std::endl;
	// }
	// printf("evaluate degeneracy: %fms\n", t_eval_degenracy.toc());
}

int main(int argc, char **argv)
{
	if (argc < 5)
	{
		printf("please intput: rosrun mloam lidar_mapper [args] \n"
			   "for example: "
			   "rosrun mloam lidar_mapper config_file 1 output_path 1 \n");
		return 1;
	}
	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc, &argv, true);

	ros::init(argc, argv, "lidar_mapper");
	ros::NodeHandle nh;

    MLOAM_RESULT_SAVE = FLAGS_result_save;
    printf("save result (0/1): %d\n", MLOAM_RESULT_SAVE);
    OUTPUT_FOLDER = FLAGS_output_path;
	UNCER_AWARE_ON = FLAGS_with_ua;
	printf("uncertainty propagation on (0/1): %d\n", UNCER_AWARE_ON);
	
	stringstream ss;
	if (UNCER_AWARE_ON)
    	ss << OUTPUT_FOLDER << "stamped_mloam_map_estimate";
	else
		ss << OUTPUT_FOLDER << "stamped_mloam_map_wo_ua_estimate";
	if (FLAGS_gf_ratio == 1.0)
		ss << ".txt";
	else
		ss << FLAGS_gf_ratio << ".txt";
	MLOAM_MAP_PATH = ss.str(); 

	std::cout << "config file: " << FLAGS_config_file << std::endl;
	readParameters(FLAGS_config_file);
	printf("Mapping as %fhz\n", 10.0 / SKIP_NUM_ODOM_PUB);

	down_size_filter_surf.setLeafSize(MAP_SURF_RES, MAP_SURF_RES, MAP_SURF_RES);
	down_size_filter_surf.trace_threshold_ = TRACE_THRESHOLD_AFTER_MAPPING;	
	down_size_filter_surf_map_cov.setLeafSize(MAP_SURF_RES, MAP_SURF_RES, MAP_SURF_RES);
	down_size_filter_surf_map_cov.trace_threshold_ = TRACE_THRESHOLD_AFTER_MAPPING;

	down_size_filter_corner.setLeafSize(MAP_CORNER_RES, MAP_CORNER_RES, MAP_CORNER_RES);
	down_size_filter_corner.trace_threshold_ = TRACE_THRESHOLD_AFTER_MAPPING;
	down_size_filter_corner_map_cov.setLeafSize(MAP_CORNER_RES, MAP_CORNER_RES, MAP_CORNER_RES);
	down_size_filter_corner_map_cov.trace_threshold_ = TRACE_THRESHOLD_AFTER_MAPPING;

	ros::Subscriber sub_laser_cloud_full_res = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud", 2, laserCloudFullResHandler);
	ros::Subscriber sub_laser_cloud_surf_last = nh.subscribe<sensor_msgs::PointCloud2>("/surf_points_less_flat", 2, laserCloudSurfLastHandler);
	ros::Subscriber sub_laser_cloud_corner_last = nh.subscribe<sensor_msgs::PointCloud2>("/corner_points_less_sharp", 2, laserCloudCornerLastHandler);
	ros::Subscriber sub_laser_odometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom", 5, laserOdometryHandler);
	ros::Subscriber sub_extrinsic = nh.subscribe<mloam_msgs::Extrinsics>("/extrinsics", 5, extrinsicsHandler);

	pub_laser_cloud_surround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
	pub_laser_cloud_map = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 2);
	pub_laser_cloud_full_res = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_registered", 2);
	pub_laser_cloud_surf_last_res = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_registered", 2);
	pub_laser_cloud_corner_last_res = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_registered", 2);

	pub_odom_aft_mapped = nh.advertise<nav_msgs::Odometry>("/laser_map", 5); // raw pose from odometry in the world
	pub_odom_aft_mapped_high_frec = nh.advertise<nav_msgs::Odometry>("/laser_map_high_frec", 5); // optimized pose in the world
	pub_laser_after_mapped_path = nh.advertise<nav_msgs::Path>("/laser_map_path", 5);
	for (int i = 0; i < laser_cloud_num; i++)
	{
		laser_cloud_surf_array_cov[i].reset(new PointICovCloud());
		laser_cloud_corner_array_cov[i].reset(new PointICovCloud());
	}

	r_ext.resize(NUM_OF_LASER);
	t_ext.resize(NUM_OF_LASER);
	pose_ext.resize(NUM_OF_LASER);
	cov_ext.resize(NUM_OF_LASER);

	laser_cloud_surf_split_cov.resize(NUM_OF_LASER);
	laser_cloud_corner_split_cov.resize(NUM_OF_LASER);

	pose_compound.resize(NUM_OF_LASER);
	cov_compound.resize(NUM_OF_LASER);

	std::thread mapping_process{process};
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	if (MLOAM_RESULT_SAVE)
	{
		save_statistics.saveMapStatistics(MLOAM_MAP_PATH,
										  OUTPUT_FOLDER + "mapping_factor.txt",
										  OUTPUT_FOLDER + "mapping_d_eigvec.txt",
										  OUTPUT_FOLDER + "mapping_pose_uncertainty",
										  laser_after_mapped_path,
										  d_factor_list,
										  d_eigvec_list,
										  cov_mapping_list);
		save_statistics.saveMapTimeStatistics(OUTPUT_FOLDER + "time_mapping.txt", total_mapping, frame_cnt);
	}

	printf("Saving laser_map cloud to /tmp/mloam_mapping_cloud.pcd\n");
	PointICovCloud::Ptr laser_cloud_map(new PointICovCloud());
	for (int i = 0; i < laser_cloud_num; i++)
	{
		*laser_cloud_map += *laser_cloud_surf_array_cov[i];
		*laser_cloud_map += *laser_cloud_corner_array_cov[i];
	}
	pcd_writer.write("/tmp/mloam_mapping_cloud.pcd", *laser_cloud_map);

	if (MLOAM_RESULT_SAVE)
	{
		PointICovCloud::Ptr laser_cloud_surf_map(new PointICovCloud());
		PointICovCloud::Ptr laser_cloud_corner_map(new PointICovCloud());
		for (int i = 0; i < laser_cloud_num; i++)
		{
			*laser_cloud_surf_map += *laser_cloud_surf_array_cov[i];
			*laser_cloud_corner_map += *laser_cloud_corner_array_cov[i];
		}
		down_size_filter_surf_map_cov.setInputCloud(laser_cloud_surf_map);
		down_size_filter_surf_map_cov.filter(*laser_cloud_surf_map);
		down_size_filter_corner_map_cov.setInputCloud(laser_cloud_corner_map);
		down_size_filter_corner_map_cov.filter(*laser_cloud_corner_map);
		pcd_writer.write("/tmp/mloam_mapping_surf_cloud.pcd", *laser_cloud_surf_map);
		pcd_writer.write("/tmp/mloam_mapping_corner_cloud.pcd", *laser_cloud_corner_map);
	}

	return 0;
}
