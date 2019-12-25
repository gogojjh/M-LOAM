#include "lidar_mapper.h"

using namespace common;

const float CUBE_SIZE = 50.0;
const float CUBE_HALF = CUBE_SIZE / 2;

int frame_cnt = 0;

double time_laser_cloud_corner_last = 0;
double time_laser_cloud_surf_last = 0;
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
PointICloud::Ptr laser_cloud_corner_last(new PointICloud());
PointICloud::Ptr laser_cloud_surf_last(new PointICloud());

// ouput: all visualble cube points
PointICloud::Ptr laser_cloud_surrond(new PointICloud());

// surround points in map to build tree
PointICloud::Ptr laser_cloud_corner_from_map(new PointICloud());
PointICloud::Ptr laser_cloud_surf_from_map(new PointICloud());

//input & output: points in one frame. local --> global
PointICloud::Ptr laser_cloud_full_res(new PointICloud());

// points in every cube
PointICloud::Ptr laser_cloud_corner_array[laser_cloud_num];
PointICloud::Ptr laser_cloud_surf_array[laser_cloud_num];

//kd-tree
pcl::KdTreeFLANN<PointI>::Ptr kdtree_corner_from_map(new pcl::KdTreeFLANN<PointI>());
pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_from_map(new pcl::KdTreeFLANN<PointI>());

// wmap_T_curr = wmap_T_odom * wodom_T_curr;
// transformation between odom's world and map's world frame
double para_pose[SIZE_POSE];
Pose pose_wmap_curr, pose_wmap_wodom, pose_wodom_curr;

// downsampling voxel grid
pcl::VoxelGrid<PointI> down_size_filter_corner;
pcl::VoxelGrid<PointI> down_size_filter_surf;

std::vector<int> point_search_ind;
std::vector<float> point_search_sq_dis;

PointI point_ori, point_sel;

nav_msgs::Path laser_after_mapped_path;

ros::Publisher pub_laser_cloud_surround, pub_laser_cloud_map, pub_laser_cloud_full_res, pub_laser_cloud_corner_last_res, pub_laser_cloud_surf_last_res;
ros::Publisher pub_odom_aft_mapped, pub_odom_aft_mapped_high_frec, pub_laser_after_mapped_path;

// extrinsics
mloam_msgs::Extrinsics extrinsics;
std::vector<Eigen::Quaterniond> q_ext;
std::vector<Eigen::Vector3d> t_ext;
std::vector<Pose> pose_ext;

std::vector<PointICloud> laser_cloud_corner_split;
std::vector<PointICloud> laser_cloud_surf_split;

// thread data buffer
std::queue<sensor_msgs::PointCloud2ConstPtr> corner_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surf_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> full_res_buf;
std::queue<nav_msgs::Odometry::ConstPtr> odometry_buf;
std::queue<mloam_msgs::ExtrinsicsConstPtr> ext_buf;
std::mutex m_buf;

FeatureExtract f_extract;

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

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_corner_last)
{
	m_buf.lock();
	corner_last_buf.push(laser_cloud_corner_last);
	m_buf.unlock();
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_surf_last)
{
	m_buf.lock();
	surf_last_buf.push(laser_cloud_surf_last);
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

void process()
{
	while(1)
	{
		while (!corner_last_buf.empty() && !surf_last_buf.empty() &&
			!full_res_buf.empty() && !ext_buf.empty() && !odometry_buf.empty())
		{
			//***************************************************************************
			// step 1: pop up subscribed data
			m_buf.lock();
			while (!odometry_buf.empty() && odometry_buf.front()->header.stamp.toSec() < corner_last_buf.front()->header.stamp.toSec())
				odometry_buf.pop();
			if (odometry_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			while (!surf_last_buf.empty() && surf_last_buf.front()->header.stamp.toSec() < corner_last_buf.front()->header.stamp.toSec())
				surf_last_buf.pop();
			if (surf_last_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			while (!full_res_buf.empty() && full_res_buf.front()->header.stamp.toSec() < corner_last_buf.front()->header.stamp.toSec())
				full_res_buf.pop();
			if (full_res_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			while (!ext_buf.empty() && ext_buf.front()->header.stamp.toSec() < corner_last_buf.front()->header.stamp.toSec())
				ext_buf.pop();
			if (ext_buf.empty())
			{
				m_buf.unlock();
				break;
			}

			time_laser_cloud_corner_last = corner_last_buf.front()->header.stamp.toSec();
			time_laser_cloud_surf_last = surf_last_buf.front()->header.stamp.toSec();
			time_laser_cloud_full_res = full_res_buf.front()->header.stamp.toSec();
			time_laser_odometry = odometry_buf.front()->header.stamp.toSec();
			time_ext = ext_buf.front()->header.stamp.toSec();

			if (time_laser_cloud_corner_last != time_laser_odometry ||
				time_laser_cloud_surf_last != time_laser_odometry ||
				time_laser_cloud_full_res != time_laser_odometry ||
				time_ext != time_laser_odometry)
			{
				printf("time corner: %f, surf: %f, full: %f, odom: %f\n",
					time_laser_cloud_corner_last, time_laser_cloud_surf_last, time_laser_cloud_full_res, time_laser_odometry);
				printf("unsync messeage!");
				m_buf.unlock();
				break;
			}

			laser_cloud_corner_last->clear();
			pcl::fromROSMsg(*corner_last_buf.front(), *laser_cloud_corner_last);
			corner_last_buf.pop();

			laser_cloud_surf_last->clear();
			pcl::fromROSMsg(*surf_last_buf.front(), *laser_cloud_surf_last);
			surf_last_buf.pop();

			laser_cloud_full_res->clear();
			pcl::fromROSMsg(*full_res_buf.front(), *laser_cloud_full_res);
			full_res_buf.pop();
			// printf("input full:%d, surf:%d, corner:%d\n", laser_cloud_full_res->size(), laser_cloud_surf_last->size(), laser_cloud_corner_last->size());

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
				ROS_INFO("Calibration is stable!");
				for (auto n = 0; n < NUM_OF_LASER; n++)
				{
					pose_ext[n].q_ = Eigen::Quaterniond(extrinsics.odoms[n].pose.pose.orientation.w,
														extrinsics.odoms[n].pose.pose.orientation.x,
														extrinsics.odoms[n].pose.pose.orientation.y,
														extrinsics.odoms[n].pose.pose.orientation.z);
					pose_ext[n].t_ = Eigen::Vector3d(extrinsics.odoms[n].pose.pose.position.x,
													 extrinsics.odoms[n].pose.pose.position.y,
													 extrinsics.odoms[n].pose.pose.position.z);
				}
			}
			ext_buf.pop();

			while(!corner_last_buf.empty())
			{
				corner_last_buf.pop();
				printf("drop lidar frame in mapping for real time performance \n");
			}
			m_buf.unlock();
			frame_cnt++;
			ROS_WARN("frame: %d", frame_cnt);
			// processMeasurements();

			//***************************************************************************
			TicToc t_whole;
			transformAssociateToMap();

			// step 2: move current map to the managed cube area
			// TODO: according to coordinate of different LiDAR with maximum points
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
							std::swap(laser_cloud_corner_array[old_cube_idx], laser_cloud_corner_array[new_cube_idx]);
							std::swap(laser_cloud_surf_array[old_cube_idx], laser_cloud_surf_array[new_cube_idx]);
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
							std::swap(laser_cloud_corner_array[old_cube_idx], laser_cloud_corner_array[new_cube_idx]);
							std::swap(laser_cloud_surf_array[old_cube_idx], laser_cloud_surf_array[new_cube_idx]);
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
							std::swap(laser_cloud_corner_array[old_cube_idx], laser_cloud_corner_array[new_cube_idx]);
							std::swap(laser_cloud_surf_array[old_cube_idx], laser_cloud_surf_array[new_cube_idx]);
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
							std::swap(laser_cloud_corner_array[old_cube_idx], laser_cloud_corner_array[new_cube_idx]);
							std::swap(laser_cloud_surf_array[old_cube_idx], laser_cloud_surf_array[new_cube_idx]);
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
							std::swap(laser_cloud_corner_array[old_cube_idx], laser_cloud_corner_array[new_cube_idx]);
							std::swap(laser_cloud_surf_array[old_cube_idx], laser_cloud_surf_array[new_cube_idx]);
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
							std::swap(laser_cloud_corner_array[old_cube_idx], laser_cloud_corner_array[new_cube_idx]);
							std::swap(laser_cloud_surf_array[old_cube_idx], laser_cloud_surf_array[new_cube_idx]);
						}
					}
				}
				center_cub_k--;
				laser_cloud_cen_depth--;
			}

			// TODO: check if in laser fov
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

			laser_cloud_corner_from_map->clear();
			laser_cloud_surf_from_map->clear();
			for (int i = 0; i < laser_cloud_valid_num; i++)
			{
				*laser_cloud_corner_from_map += *laser_cloud_corner_array[laser_cloud_valid_ind[i]];
				*laser_cloud_surf_from_map += *laser_cloud_surf_array[laser_cloud_valid_ind[i]];
			}
			int laser_cloud_corner_from_map_num = laser_cloud_corner_from_map->points.size();
			int laser_cloud_surf_from_map_num = laser_cloud_surf_from_map->points.size();

			PointICloud::Ptr laser_cloud_corner_stack(new PointICloud());
			down_size_filter_corner.setInputCloud(laser_cloud_corner_last);
			down_size_filter_corner.filter(*laser_cloud_corner_stack);
			int laser_cloud_corner_stack_num = laser_cloud_corner_stack->points.size();

			PointICloud::Ptr laser_cloud_surf_stack(new PointICloud());
			down_size_filter_surf.setInputCloud(laser_cloud_surf_last);
			down_size_filter_surf.filter(*laser_cloud_surf_stack);
			int laser_cloud_surf_stack_num = laser_cloud_surf_stack->points.size();

			//**************************************************************
			// noisy point filtering
			for (auto n = 0; n < NUM_OF_LASER; n++)
			{
				laser_cloud_corner_split[n].clear();
				laser_cloud_surf_split[n].clear();
			}
			for (auto point_ori: laser_cloud_corner_stack->points)
			{
				int idx = int(point_ori.intensity); // indicate the lidar id
				PointI point_sel;
				Eigen::Matrix3d cov_po = Eigen::Matrix3d::Zero();
				pointAssociateToMap(point_ori, point_sel, pose_ext[idx].inverse());
				evalPointUncertainty(idx, point_sel, cov_po);
				if (cov_po.norm() < NORM_THRESHOLD)
				{
					laser_cloud_corner_split[idx].push_back(point_ori);
					PointIWithCov point_cov(point_ori, cov_po.cast<float>());
				}
			}
			for (auto point_ori: laser_cloud_surf_stack->points)
			{
				int idx = int(point_ori.intensity); // indicate the lidar id
				PointI point_sel;
				Eigen::Matrix3d cov_po = Eigen::Matrix3d::Zero();
				pointAssociateToMap(point_ori, point_sel, pose_ext[idx].inverse());
				evalPointUncertainty(idx, point_sel, cov_po);
				if (cov_po.norm() < NORM_THRESHOLD)
				{
					laser_cloud_surf_split[idx].push_back(point_ori);
					PointIWithCov point_cov(point_ori, cov_po.cast<float>());
				}
			}

			//***************************************************************************
			// step 3: perform scan-to-map optimization
			printf("map prepare time %fms\n", t_shift.toc());
			printf("map corner num:%d, surf num:%d\n", laser_cloud_corner_from_map_num, laser_cloud_surf_from_map_num);
			if (laser_cloud_corner_from_map_num > 10 && laser_cloud_surf_from_map_num > 50)
			{
				TicToc t_opt;
				TicToc t_tree;
				kdtree_corner_from_map->setInputCloud(laser_cloud_corner_from_map);
				kdtree_surf_from_map->setInputCloud(laser_cloud_surf_from_map);
				printf("build tree time %fms\n", t_tree.toc());

				std::vector<double *> para_ids;
				std::vector<ceres::internal::ResidualBlock *> res_ids;
				for (int iter_cnt = 0; iter_cnt < 2; iter_cnt++)
				{
					printf("********************************\n");
					ceres::Problem problem;
					ceres::Solver::Summary summary;
					ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

					ceres::Solver::Options options;
					options.linear_solver_type = ceres::DENSE_QR;
					options.max_num_iterations = 20;
					options.minimizer_progress_to_stdout = false;
					options.check_gradients = false;
					options.gradient_check_relative_precision = 1e-4;

					vector2Double();

					PoseLocalParameterization *local_parameterization = new PoseLocalParameterization();
					local_parameterization->setParameter();
					problem.AddParameterBlock(para_pose, SIZE_POSE, local_parameterization);

					// ******************************************************
					// ROS_INFO("mloam modules");
					std::vector<double> info_weight(NUM_OF_LASER, 0);
					info_weight[0] = 1.0;
					info_weight[1] = 1.0;
					int corner_num = 0;
					int surf_num = 0;
					TicToc t_prepare;
					for (auto n = 0; n < NUM_OF_LASER; n++)
					{
						// if ((n != IDX_REF) && (extrinsics.status)) continue;
						// if (n != IDX_REF) continue;
						PointICloud &laser_cloud_corner_points = laser_cloud_corner_split[n];
						PointICloud &laser_cloud_surf_points = laser_cloud_surf_split[n];
						// PointICloud &laser_cloud_corner_points = *laser_cloud_corner_stack;
						// PointICloud &laser_cloud_surf_points = *laser_cloud_surf_stack;

						TicToc t_data;
						int n_neigh = 5;
						Pose pose_local = pose_wmap_curr;
						std::vector<PointPlaneFeature> corner_map_features, surf_map_features;
						f_extract.extractCornerFromMap(kdtree_corner_from_map, *laser_cloud_corner_from_map,
							laser_cloud_corner_points, pose_local, corner_map_features, n_neigh, false);
						f_extract.extractSurfFromMap(kdtree_surf_from_map, *laser_cloud_surf_from_map,
							laser_cloud_surf_points, pose_local, surf_map_features, n_neigh, false);
						corner_num += corner_map_features.size() / 2;
						surf_num += surf_map_features.size();
						// printf("mapping data assosiation time %fms\n", t_data.toc());

						for (auto &feature: corner_map_features)
						{
							const double &s = feature.score_;
	                        const Eigen::Vector3d &p_data = feature.point_;
	                        const Eigen::Vector4d &coeff_ref = feature.coeffs_;
							ceres::CostFunction *cost_function = LidarMapPlaneNormFactor::Create(p_data, coeff_ref, s, info_weight[n]);
							problem.AddResidualBlock(cost_function, loss_function, para_pose);
						}

						for (auto &feature: surf_map_features)
						{
							const double &s = feature.score_;
	                        const Eigen::Vector3d &p_data = feature.point_;
	                        const Eigen::Vector4d &coeff_ref = feature.coeffs_;
							ceres::CostFunction *cost_function = LidarMapPlaneNormFactor::Create(p_data, coeff_ref, s, info_weight[n]);
							problem.AddResidualBlock(cost_function, loss_function, para_pose);
						}
					}
					// printf("prepare ceres data %fms\n", t_prepare.toc());
					printf("corner num %d, used corner num %d\n", laser_cloud_corner_stack_num, corner_num);
					printf("surf num %d, used surf num %d\n", laser_cloud_surf_stack_num, surf_num);
					// printf("residual block size: %d\n", problem.NumResidualBlocks());
					double cost = 0.0;
					problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
					// printf("cost: %f\n", cost);

					// ******************************************************
					TicToc t_solver;
					ceres::Solve(options, &problem, &summary);
					std::cout << summary.BriefReport() << std::endl;
					printf("mapping solver time %fms\n", t_solver.toc());

					//printf("time %f \n", time_laser_odometry);
					//printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
					//printf("result q %f %f %f %f result t %f %f %f\n", para_pose[3], para_pose[0], para_pose[1], para_pose[2],
					//	   para_pose[4], para_pose[5], para_pose[6]);
					double2Vector();
					std::cout << iter_cnt << "th result: " << pose_wmap_curr << std::endl;
				}
				printf("mapping optimization time %fms\n", t_opt.toc());
			}
			else
			{
				ROS_WARN("time Map corner and surf num are not enough");
			}
			transformUpdate();

			// add new map points to the cubes
			TicToc t_add;
			for (auto n = 0; n < NUM_OF_LASER; n++)
			{
				PointICloud &laser_cloud_corner_points = laser_cloud_corner_split[n];
				PointICloud &laser_cloud_surf_points = laser_cloud_surf_split[n];

				// move the corner points from the lastest frame to different cubes
				for (int i = 0; i < laser_cloud_corner_points.size(); i++)
				{
					pointAssociateToMap(laser_cloud_corner_points.points[i], point_sel, pose_wmap_curr);
					int cube_i = int((point_sel.x + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_width;
					int cube_j = int((point_sel.y + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_height;
					int cube_k = int((point_sel.z + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_depth;
					if (point_sel.x + CUBE_HALF < 0) cube_i--;
					if (point_sel.y + CUBE_HALF < 0) cube_j--;
					if (point_sel.z + CUBE_HALF < 0) cube_k--;

					if (cube_i >= 0 && cube_i < laser_cloud_width &&
						cube_j >= 0 && cube_j < laser_cloud_height &&
						cube_k >= 0 && cube_k < laser_cloud_depth)
					{
						int cur_cube_idx = toCubeIndex(cube_i, cube_j, cube_k);
						laser_cloud_corner_array[cur_cube_idx]->push_back(point_sel);
					}
				}

				// move the surf points from the lastest frame to different cubes
				for (int i = 0; i < laser_cloud_surf_points.size(); i++)
				{
					pointAssociateToMap(laser_cloud_surf_points.points[i], point_sel, pose_wmap_curr);
					int cube_i = int((point_sel.x + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_width;
					int cube_j = int((point_sel.y + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_height;
					int cube_k = int((point_sel.z + CUBE_HALF) / CUBE_SIZE) + laser_cloud_cen_depth;

					if (point_sel.x + CUBE_HALF < 0) cube_i--;
					if (point_sel.y + CUBE_HALF < 0) cube_j--;
					if (point_sel.z + CUBE_HALF < 0) cube_k--;

					if (cube_i >= 0 && cube_i < laser_cloud_width &&
						cube_j >= 0 && cube_j < laser_cloud_height &&
						cube_k >= 0 && cube_k < laser_cloud_depth)
					{
						int cur_cube_idx = toCubeIndex(cube_i, cube_j, cube_k);
						laser_cloud_surf_array[cur_cube_idx]->push_back(point_sel);
					}
				}
			}
			printf("add points time %fms\n", t_add.toc());

			// downsample the map (all map points including optimization or not optimization)
			TicToc t_filter;
			for (int i = 0; i < laser_cloud_valid_num; i++)
			{
				int ind = laser_cloud_valid_ind[i];
				PointICloud::Ptr tmpCorner(new PointICloud());
				down_size_filter_corner.setInputCloud(laser_cloud_corner_array[ind]);
				down_size_filter_corner.filter(*tmpCorner);
				laser_cloud_corner_array[ind] = tmpCorner;

				PointICloud::Ptr tmpSurf(new PointICloud());
				down_size_filter_surf.setInputCloud(laser_cloud_surf_array[ind]);
				down_size_filter_surf.filter(*tmpSurf);
				laser_cloud_surf_array[ind] = tmpSurf;
			}
			printf("filter time %fms \n", t_filter.toc());

			//**************************************************************
			// publish surround map (use for optimization) for every 5 frame
			TicToc t_pub;
			if (frame_cnt % 5 == 0)
			{
				laser_cloud_surrond->clear();
				for (int i = 0; i < laser_cloud_surround_num; i++)
				{
					int ind = laser_cloud_surrond_ind[i];
					*laser_cloud_surrond += *laser_cloud_corner_array[ind];
					*laser_cloud_surrond += *laser_cloud_surf_array[ind];
				}
				sensor_msgs::PointCloud2 laser_cloud_surround_msg;
				pcl::toROSMsg(*laser_cloud_surrond, laser_cloud_surround_msg);
				laser_cloud_surround_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
				laser_cloud_surround_msg.header.frame_id = "/world";
				pub_laser_cloud_surround.publish(laser_cloud_surround_msg);
				printf("size of surround map: %d\n", laser_cloud_surrond->size());
			}

			// publish valid map for every 5 frame
			if (frame_cnt % 20 == 0)
			{
				PointICloud laser_cloud_map;
				for (int i = 0; i < laser_cloud_num; i++)
				{
					laser_cloud_map += *laser_cloud_corner_array[i];
					laser_cloud_map += *laser_cloud_surf_array[i];
				}
				sensor_msgs::PointCloud2 laser_cloud_msg;
				pcl::toROSMsg(laser_cloud_map, laser_cloud_msg);
				laser_cloud_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
				laser_cloud_msg.header.frame_id = "/world";
				pub_laser_cloud_map.publish(laser_cloud_msg);
				printf("size of cloud map: %d\n", laser_cloud_map.size());
			}

			// publish registrated laser cloud
			int laser_cloud_full_res_name = laser_cloud_full_res->points.size();
			for (int i = 0; i < laser_cloud_full_res_name; i++)
			{
				pointAssociateToMap(laser_cloud_full_res->points[i], laser_cloud_full_res->points[i], pose_wmap_curr);
			}
			sensor_msgs::PointCloud2 laser_cloud_full_res_msg;
			pcl::toROSMsg(*laser_cloud_full_res, laser_cloud_full_res_msg);
			laser_cloud_full_res_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
			laser_cloud_full_res_msg.header.frame_id = "/world";
			pub_laser_cloud_full_res.publish(laser_cloud_full_res_msg);

			*laser_cloud_corner_last = laser_cloud_corner_split[IDX_REF];
			for (int i = 0; i < laser_cloud_corner_last->points.size(); i++)
			{
				pointAssociateToMap(laser_cloud_corner_last->points[i], laser_cloud_corner_last->points[i], pose_wmap_curr);
			}
			sensor_msgs::PointCloud2 laser_cloud_corner_last_msg;
			pcl::toROSMsg(*laser_cloud_corner_last, laser_cloud_corner_last_msg);
			laser_cloud_corner_last_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
			laser_cloud_corner_last_msg.header.frame_id = "/world";
			pub_laser_cloud_corner_last_res.publish(laser_cloud_corner_last_msg);

			*laser_cloud_surf_last = laser_cloud_surf_split[IDX_REF];
			for (int i = 0; i < laser_cloud_surf_last->points.size(); i++)
			{
				pointAssociateToMap(laser_cloud_surf_last->points[i], laser_cloud_surf_last->points[i], pose_wmap_curr);
			}
			sensor_msgs::PointCloud2 laser_cloud_surf_last_msg;
			pcl::toROSMsg(*laser_cloud_surf_last, laser_cloud_surf_last_msg);
			laser_cloud_surf_last_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
			laser_cloud_surf_last_msg.header.frame_id = "/world";
			pub_laser_cloud_surf_last_res.publish(laser_cloud_surf_last_msg);

			printf("mapping pub time %fms \n", t_pub.toc());
			printf("whole mapping time %fms +++++\n", t_whole.toc());

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
			pub_odom_aft_mapped.publish(odom_aft_mapped);

			geometry_msgs::PoseStamped laser_after_mapped_pose;
			laser_after_mapped_pose.header = odom_aft_mapped.header;
			laser_after_mapped_pose.pose = odom_aft_mapped.pose.pose;
			laser_after_mapped_path.header.stamp = odom_aft_mapped.header.stamp;
			laser_after_mapped_path.header.frame_id = "/world";
			laser_after_mapped_path.poses.push_back(laser_after_mapped_pose);
			pub_laser_after_mapped_path.publish(laser_after_mapped_path);
			publishTF(odom_aft_mapped);

			if (MLOAM_RESULT_SAVE)
			{
				std::ofstream fout(MLOAM_MAP_PATH.c_str(), std::ios::out);
				fout.precision(5);
				for (size_t i = 0; i < laser_after_mapped_path.poses.size(); i++)
				{
					geometry_msgs::PoseStamped &laser_pose = laser_after_mapped_path.poses[i];
					fout << laser_pose.header.stamp.toSec() << " "
						<< laser_pose.pose.position.x << " "
						<< laser_pose.pose.position.y << " "
						<< laser_pose.pose.position.z << " "
						<< laser_pose.pose.orientation.x << " "
						<< laser_pose.pose.orientation.y << " "
						<< laser_pose.pose.orientation.z << " "
						<< laser_pose.pose.orientation.w << std::endl;
				}
				fout.close();
			}
			// std::cout << "pose_wmap_curr: " << pose_wmap_curr << std::endl;
			printf("\n");
		}
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_mapper");
	ros::NodeHandle nh;
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	string config_file = argv[1];
	readParameters(config_file);

	// set resolution
	float lineRes = 0.2;
	float planeRes = 0.4;
	printf("line resolution:%f, plane resolution:%f\n", lineRes, planeRes);
	down_size_filter_corner.setLeafSize(lineRes, lineRes,lineRes);
	down_size_filter_surf.setLeafSize(planeRes, planeRes, planeRes);

	ros::Subscriber sub_laser_cloud_full_res = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud", 100, laserCloudFullResHandler);
	ros::Subscriber sub_laser_cloud_corner_last = nh.subscribe<sensor_msgs::PointCloud2>("/corner_points_less_sharp", 100, laserCloudCornerLastHandler);
	ros::Subscriber sub_laser_cloud_surf_last = nh.subscribe<sensor_msgs::PointCloud2>("/surf_points_less_flat", 100, laserCloudSurfLastHandler);
	ros::Subscriber sub_laser_odometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_0", 100, laserOdometryHandler);
	ros::Subscriber sub_extrinsic = nh.subscribe<mloam_msgs::Extrinsics>("/extrinsics", 100, extrinsicsHandler);

	pub_laser_cloud_surround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);
	pub_laser_cloud_map = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
	pub_laser_cloud_full_res = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_registered", 100);
	pub_laser_cloud_corner_last_res = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_registered", 100);
	pub_laser_cloud_surf_last_res = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_registered", 100);

	pub_odom_aft_mapped = nh.advertise<nav_msgs::Odometry>("/laser_map", 100); // raw pose from odometry in the world
	pub_odom_aft_mapped_high_frec = nh.advertise<nav_msgs::Odometry>("/laser_map_high_frec", 100); // optimized pose in the world
	pub_laser_after_mapped_path = nh.advertise<nav_msgs::Path>("/laser_map_path", 100);
	for (int i = 0; i < laser_cloud_num; i++)
	{
		laser_cloud_corner_array[i].reset(new PointICloud());
		laser_cloud_surf_array[i].reset(new PointICloud());
	}

	q_ext.resize(NUM_OF_LASER);
	t_ext.resize(NUM_OF_LASER);
	pose_ext.resize(NUM_OF_LASER);

	laser_cloud_corner_split.resize(NUM_OF_LASER);
	laser_cloud_surf_split.resize(NUM_OF_LASER);

	std::thread mapping_process{process};
	ros::spin();

	return 0;
}
