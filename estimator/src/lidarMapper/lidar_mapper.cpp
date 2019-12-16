#include "lidar_mapper.h"

using namespace common;

int frame_count = 0;

double time_laser_cloud_corner_last = 0;
double time_laser_cloud_surf_last = 0;
double time_laser_cloud_full_res = 0;
double time_laser_odometry = 0;
double time_ext = 0;

// the cude size for mapping
int laser_cloud_cen_width = 10;
int laser_cloud_cen_height = 10;
int laser_cloud_cen_depth = 5;

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

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);

std::queue<sensor_msgs::PointCloud2ConstPtr> corner_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surf_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> full_res_buf;
std::queue<nav_msgs::Odometry::ConstPtr> odometry_buf;
std::queue<mloam_msgs::ExtrinsicsConstPtr> ext_buf;
std::mutex m_buf;

pcl::VoxelGrid<PointI> down_size_filter_corner;
pcl::VoxelGrid<PointI> down_size_filter_surf;

std::vector<int> point_search_ind;
std::vector<float> point_search_sq_dis;

PointI point_ori, point_sel;

ros::Publisher pub_laser_cloud_surround, pub_laser_cloud_map, pub_laser_cloud_full_res, pub_odom_aft_mapped, pub_odom_aft_mapped_high_frec, pub_laser_after_mapped_path;

nav_msgs::Path laser_after_mapped_path;

mloam_msgs::Extrinsics extrinsics;
std::vector<Eigen::Quaterniond> q_ext;
std::vector<Eigen::Vector3d> t_ext;
std::vector<Pose> pose_ext;

// set initial guess
void transformAssociateToMap()
{
	q_w_curr = q_wmap_wodom * q_wodom_curr;
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void transformUpdate()
{
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

void pointAssociateToMap(PointI const *const pi, PointI *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
	//po->intensity = 1.0;
}

void pointAssociateTobeMapped(PointI const *const pi, PointI *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
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
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laser_odometry)
{
	m_buf.lock();
	odometry_buf.push(laser_odometry);
	m_buf.unlock();

	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = laser_odometry->pose.pose.orientation.x;
	q_wodom_curr.y() = laser_odometry->pose.pose.orientation.y;
	q_wodom_curr.z() = laser_odometry->pose.pose.orientation.z;
	q_wodom_curr.w() = laser_odometry->pose.pose.orientation.w;
	t_wodom_curr.x() = laser_odometry->pose.pose.position.x;
	t_wodom_curr.y() = laser_odometry->pose.pose.position.y;
	t_wodom_curr.z() = laser_odometry->pose.pose.position.z;

	Eigen::Quaterniond q_w_curr_ini = q_wmap_wodom * q_wodom_curr;
	Eigen::Vector3d t_w_curr_ini = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
	nav_msgs::Odometry odom_aft_mapped;
	odom_aft_mapped.header.frame_id = "/world";
	odom_aft_mapped.child_frame_id = "/aft_mapped";
	odom_aft_mapped.header.stamp = laser_odometry->header.stamp;
	odom_aft_mapped.pose.pose.orientation.x = q_w_curr_ini.x();
	odom_aft_mapped.pose.pose.orientation.y = q_w_curr_ini.y();
	odom_aft_mapped.pose.pose.orientation.z = q_w_curr_ini.z();
	odom_aft_mapped.pose.pose.orientation.w = q_w_curr_ini.w();
	odom_aft_mapped.pose.pose.position.x = t_w_curr_ini.x();
	odom_aft_mapped.pose.pose.position.y = t_w_curr_ini.y();
	odom_aft_mapped.pose.pose.position.z = t_w_curr_ini.z();
	pub_odom_aft_mapped_high_frec.publish(odom_aft_mapped); // publish (k-1)th oldest map * kth newest odom
}

//TODO: main process
void process()
{
	while(1)
	{
		while (!corner_last_buf.empty() && !surf_last_buf.empty() &&
			!full_res_buf.empty() && !ext_buf.empty() && !odometry_buf.empty())
		{
			// step1: pop up pointcloud

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
			// printf("input cloud full:%d, surf:%d, corner:%d\n", laser_cloud_full_res->size(), laser_cloud_surf_last->size(), laser_cloud_corner_last->size());

			q_wodom_curr.x() = odometry_buf.front()->pose.pose.orientation.x;
			q_wodom_curr.y() = odometry_buf.front()->pose.pose.orientation.y;
			q_wodom_curr.z() = odometry_buf.front()->pose.pose.orientation.z;
			q_wodom_curr.w() = odometry_buf.front()->pose.pose.orientation.w;
			t_wodom_curr.x() = odometry_buf.front()->pose.pose.position.x;
			t_wodom_curr.y() = odometry_buf.front()->pose.pose.position.y;
			t_wodom_curr.z() = odometry_buf.front()->pose.pose.position.z;
			odometry_buf.pop();

			extrinsics = *ext_buf.front();
			if (!extrinsics.converage)
			{
				ROS_INFO("Calibration is stable!");
				for (auto n = 0; n < NUM_OF_LASER; n++)
				{
					q_ext[n] = Eigen::Quaterniond::Identity();
					t_ext[n] = Eigen::Vector3d::Zero();
					q_ext[n].x() = extrinsics.odoms[n].pose.pose.orientation.x;
					q_ext[n].y() = extrinsics.odoms[n].pose.pose.orientation.y;
					q_ext[n].z() = extrinsics.odoms[n].pose.pose.orientation.z;
					q_ext[n].w() = extrinsics.odoms[n].pose.pose.orientation.w;
					t_ext[n].x() = extrinsics.odoms[n].pose.pose.position.x;
					t_ext[n].y() = extrinsics.odoms[n].pose.pose.position.y;
					t_ext[n].z() = extrinsics.odoms[n].pose.pose.position.z;
					pose_ext[n] = Pose(q_ext[n], t_ext[n]);
				}
			}
			ext_buf.pop();

			while(!corner_last_buf.empty())
			{
				corner_last_buf.pop();
				printf("drop lidar frame in mapping for real time performance \n");
			}
			m_buf.unlock();

			TicToc t_whole;

			transformAssociateToMap();

			// set map cube for optimization
			TicToc t_shift;
			int center_cub_i = int((t_w_curr.x() + 25.0) / 50.0) + laser_cloud_cen_width;
			int center_cub_j = int((t_w_curr.y() + 25.0) / 50.0) + laser_cloud_cen_height;
			int center_cub_k = int((t_w_curr.z() + 25.0) / 50.0) + laser_cloud_cen_depth;
			if (t_w_curr.x() + 25.0 < 0) center_cub_i--;
			if (t_w_curr.y() + 25.0 < 0) center_cub_j--;
			if (t_w_curr.z() + 25.0 < 0) center_cub_k--;

			// TODO:
			{
			while (center_cub_i < 3)
			{
				for (int j = 0; j < laser_cloud_height; j++)
				{
					for (int k = 0; k < laser_cloud_depth; k++)
					{
						int i = laser_cloud_width - 1;
						PointICloud::Ptr laser_cloud_cube_corner_pointer =
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						PointICloud::Ptr laser_cloud_cube_surf_pointer =
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						for (; i >= 1; i--)
						{
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_corner_array[i - 1 + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_surf_array[i - 1 + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						}
						laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_corner_pointer;
						laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_surf_pointer;
						laser_cloud_cube_corner_pointer->clear();
						laser_cloud_cube_surf_pointer->clear();
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
						int i = 0;
						PointICloud::Ptr laser_cloud_cube_corner_pointer =
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						PointICloud::Ptr laser_cloud_cube_surf_pointer =
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						for (; i < laser_cloud_width - 1; i++)
						{
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_corner_array[i + 1 + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_surf_array[i + 1 + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						}
						laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_corner_pointer;
						laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_surf_pointer;
						laser_cloud_cube_corner_pointer->clear();
						laser_cloud_cube_surf_pointer->clear();
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
						int j = laser_cloud_height - 1;
						PointICloud::Ptr laser_cloud_cube_corner_pointer =
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						PointICloud::Ptr laser_cloud_cube_surf_pointer =
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						for (; j >= 1; j--)
						{
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_corner_array[i + laser_cloud_width * (j - 1) + laser_cloud_width * laser_cloud_height * k];
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_surf_array[i + laser_cloud_width * (j - 1) + laser_cloud_width * laser_cloud_height * k];
						}
						laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_corner_pointer;
						laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_surf_pointer;
						laser_cloud_cube_corner_pointer->clear();
						laser_cloud_cube_surf_pointer->clear();
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
						int j = 0;
						PointICloud::Ptr laser_cloud_cube_corner_pointer =
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						PointICloud::Ptr laser_cloud_cube_surf_pointer =
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						for (; j < laser_cloud_height - 1; j++)
						{
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_corner_array[i + laser_cloud_width * (j + 1) + laser_cloud_width * laser_cloud_height * k];
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_surf_array[i + laser_cloud_width * (j + 1) + laser_cloud_width * laser_cloud_height * k];
						}
						laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_corner_pointer;
						laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_surf_pointer;
						laser_cloud_cube_corner_pointer->clear();
						laser_cloud_cube_surf_pointer->clear();
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
						int k = laser_cloud_depth - 1;
						PointICloud::Ptr laser_cloud_cube_corner_pointer =
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						PointICloud::Ptr laser_cloud_cube_surf_pointer =
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						for (; k >= 1; k--)
						{
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * (k - 1)];
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * (k - 1)];
						}
						laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_corner_pointer;
						laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_surf_pointer;
						laser_cloud_cube_corner_pointer->clear();
						laser_cloud_cube_surf_pointer->clear();
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
						int k = 0;
						PointICloud::Ptr laser_cloud_cube_corner_pointer =
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						PointICloud::Ptr laser_cloud_cube_surf_pointer =
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k];
						for (; k < laser_cloud_depth - 1; k++)
						{
							laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * (k + 1)];
							laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
								laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * (k + 1)];
						}
						laser_cloud_corner_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_corner_pointer;
						laser_cloud_surf_array[i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k] =
							laser_cloud_cube_surf_pointer;
						laser_cloud_cube_corner_pointer->clear();
						laser_cloud_cube_surf_pointer->clear();
					}
				}
				center_cub_k--;
				laser_cloud_cen_depth--;
			}
			}

			int laser_cloud_valid_num = 0;
			int laser_cloud_surround_num = 0;

			for (int i = center_cub_i - 2; i <= center_cub_i + 2; i++)
			{
				for (int j = center_cub_j - 2; j <= center_cub_j + 2; j++)
				{
					for (int k = center_cub_k - 1; k <= center_cub_k + 1; k++)
					{
						if ((i >= 0 && i < laser_cloud_width) &&
							(j >= 0 && j < laser_cloud_height) &&
							(k >= 0 && k < laser_cloud_depth))
						{
							laser_cloud_valid_ind[laser_cloud_valid_num] = i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k;
							laser_cloud_valid_num++;
							laser_cloud_surrond_ind[laser_cloud_surround_num] = i + laser_cloud_width * j + laser_cloud_width * laser_cloud_height * k;
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
			down_size_filter_corner.setInputCloud(laser_cloud_surf_last);
			down_size_filter_corner.filter(*laser_cloud_corner_stack);
			int laser_cloud_corner_stack_num = laser_cloud_corner_stack->points.size();

			PointICloud::Ptr laser_cloud_surf_stack(new PointICloud());
			down_size_filter_surf.setInputCloud(laser_cloud_surf_last);
			down_size_filter_surf.filter(*laser_cloud_surf_stack);
			int laser_cloud_surf_stack_num = laser_cloud_surf_stack->points.size();

			// TODO: perform scan-to-map optimization
			printf("map prepare time %fms\n", t_shift.toc());
			printf("map corner num:%d, surf num:%d\n", laser_cloud_corner_from_map_num, laser_cloud_surf_from_map_num);
			if (laser_cloud_corner_from_map_num > 10 && laser_cloud_surf_from_map_num > 50)
			{
				TicToc t_opt;
				TicToc t_tree;
				kdtree_corner_from_map->setInputCloud(laser_cloud_corner_from_map);
				kdtree_surf_from_map->setInputCloud(laser_cloud_surf_from_map);
				printf("build tree time %fms \n", t_tree.toc());
				for (int iterCount = 0; iterCount < 2; iterCount++)
				{
					//ceres::LossFunction *loss_function = NULL;
					ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
					ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
					ceres::Problem::Options problem_options;

					ceres::Problem problem(problem_options);
					problem.AddParameterBlock(parameters, 4, q_parameterization);
					problem.AddParameterBlock(parameters + 4, 3);

					// corner map
					TicToc t_data;
					int corner_num = 0;

					for (int i = 0; i < laser_cloud_corner_stack_num; i++)
					{
						point_ori = laser_cloud_corner_stack->points[i];
						//double sqrtDis = point_ori.x * point_ori.x + point_ori.y * point_ori.y + point_ori.z * point_ori.z;
						pointAssociateToMap(&point_ori, &point_sel);
						kdtree_corner_from_map->nearestKSearch(point_sel, 5, point_search_ind, point_search_sq_dis);

						if (point_search_sq_dis[4] < 1.0)
						{
							std::vector<Eigen::Vector3d> nearCorners;
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laser_cloud_corner_from_map->points[point_search_ind[j]].x,
													laser_cloud_corner_from_map->points[point_search_ind[j]].y,
													laser_cloud_corner_from_map->points[point_search_ind[j]].z);
								center = center + tmp;
								nearCorners.push_back(tmp);
							}
							center = center / 5.0;

							Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
							for (int j = 0; j < 5; j++)
							{
								Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
								covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
							}

							Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

							// if is indeed line feature
							// note Eigen library sort eigenvalues in increasing order
							Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
							Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
							if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
							{
								Eigen::Vector3d point_on_line = center;
								Eigen::Vector3d point_a, point_b;
								point_a = 0.1 * unit_direction + point_on_line;
								point_b = -0.1 * unit_direction + point_on_line;

								ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								corner_num++;
							}
						}
						/*
						else if(point_search_sq_dis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laser_cloud_corner_from_map->points[point_search_ind[j]].x,
													laser_cloud_corner_from_map->points[point_search_ind[j]].y,
													laser_cloud_corner_from_map->points[point_search_ind[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;
							Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}

					// surf map
					int surf_num = 0;
					for (int i = 0; i < laser_cloud_surf_stack_num; i++)
					{
						point_ori = laser_cloud_surf_stack->points[i];
						//double sqrtDis = point_ori.x * point_ori.x + point_ori.y * point_ori.y + point_ori.z * point_ori.z;
						pointAssociateToMap(&point_ori, &point_sel);
						kdtree_surf_from_map->nearestKSearch(point_sel, 5, point_search_ind, point_search_sq_dis); // find the nearest 5 points
						Eigen::Matrix<double, 5, 3> matA0;
						Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
						if (point_search_sq_dis[4] < 1.0)
						{
							for (int j = 0; j < 5; j++)
							{
								matA0(j, 0) = laser_cloud_surf_from_map->points[point_search_ind[j]].x;
								matA0(j, 1) = laser_cloud_surf_from_map->points[point_search_ind[j]].y;
								matA0(j, 2) = laser_cloud_surf_from_map->points[point_search_ind[j]].z;
								//printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
							}
							// find the norm of plane
							Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
							double negative_OA_dot_norm = 1 / norm.norm();
							norm.normalize();

							// Here n(pa, pb, pc) is unit norm of plane
							bool planeValid = true;
							for (int j = 0; j < 5; j++)
							{
								// if OX * n > 0.2, then plane is not fit well
								if (fabs(norm(0) * laser_cloud_surf_from_map->points[point_search_ind[j]].x +
										 norm(1) * laser_cloud_surf_from_map->points[point_search_ind[j]].y +
										 norm(2) * laser_cloud_surf_from_map->points[point_search_ind[j]].z + negative_OA_dot_norm) > 0.2)
								{
									planeValid = false;
									break;
								}
							}
							Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
							if (planeValid)
							{
								ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								surf_num++;
							}
						}
						/*
						else if(point_search_sq_dis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laser_cloud_surf_from_map->points[point_search_ind[j]].x,
													laser_cloud_surf_from_map->points[point_search_ind[j]].y,
													laser_cloud_surf_from_map->points[point_search_ind[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;
							Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}

					//printf("corner num %d used corner num %d \n", laser_cloud_corner_stack_num, corner_num);
					//printf("surf num %d used surf num %d \n", laser_cloud_surf_stack_num, surf_num);
					printf("mapping data assosiation time %fms \n", t_data.toc());

					TicToc t_solver;
					ceres::Solver::Options options;
					options.linear_solver_type = ceres::DENSE_QR;
					options.max_num_iterations = 4;
					options.minimizer_progress_to_stdout = false;
					options.check_gradients = false;
					options.gradient_check_relative_precision = 1e-4;
					ceres::Solver::Summary summary;
					ceres::Solve(options, &problem, &summary);
					printf("mapping solver time %fms \n", t_solver.toc());

					//printf("time %f \n", time_laser_odometry);
					//printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
					//printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
					//	   parameters[4], parameters[5], parameters[6]);
				}
				printf("mapping optimization time %fms\n", t_opt.toc());
			}
			else
			{
				ROS_WARN("time Map corner and surf num are not enough");
			}
			transformUpdate();

			TicToc t_add;
			for (int i = 0; i < laser_cloud_corner_stack_num; i++)
			{
				pointAssociateToMap(&laser_cloud_corner_stack->points[i], &point_sel);

				int cubeI = int((point_sel.x + 25.0) / 50.0) + laser_cloud_cen_width;
				int cubeJ = int((point_sel.y + 25.0) / 50.0) + laser_cloud_cen_height;
				int cubeK = int((point_sel.z + 25.0) / 50.0) + laser_cloud_cen_height;

				if (point_sel.x + 25.0 < 0) cubeI--;
				if (point_sel.y + 25.0 < 0) cubeJ--;
				if (point_sel.z + 25.0 < 0) cubeK--;

				if (cubeI >= 0 && cubeI < laser_cloud_width &&
					cubeJ >= 0 && cubeJ < laser_cloud_height &&
					cubeK >= 0 && cubeK < laser_cloud_depth)
				{
					int cubeInd = cubeI + laser_cloud_width * cubeJ + laser_cloud_width * laser_cloud_height * cubeK;
					laser_cloud_corner_array[cubeInd]->push_back(point_sel);
				}
			}

			for (int i = 0; i < laser_cloud_surf_stack_num; i++)
			{
				pointAssociateToMap(&laser_cloud_surf_stack->points[i], &point_sel);

				int cubeI = int((point_sel.x + 25.0) / 50.0) + laser_cloud_cen_width;
				int cubeJ = int((point_sel.y + 25.0) / 50.0) + laser_cloud_cen_height;
				int cubeK = int((point_sel.z + 25.0) / 50.0) + laser_cloud_cen_height;

				if (point_sel.x + 25.0 < 0) cubeI--;
				if (point_sel.y + 25.0 < 0) cubeJ--;
				if (point_sel.z + 25.0 < 0) cubeK--;

				if (cubeI >= 0 && cubeI < laser_cloud_width &&
					cubeJ >= 0 && cubeJ < laser_cloud_height &&
					cubeK >= 0 && cubeK < laser_cloud_depth)
				{
					int cubeInd = cubeI + laser_cloud_width * cubeJ + laser_cloud_width * laser_cloud_height * cubeK;
					laser_cloud_surf_array[cubeInd]->push_back(point_sel);
				}
			}
			printf("add points time %fms\n", t_add.toc());


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

			//publish surround map for every 5 frame
			TicToc t_pub;
			if (frame_count % 5 == 0)
			{
				laser_cloud_surrond->clear();
				for (int i = 0; i < laser_cloud_surround_num; i++)
				{
					int ind = laser_cloud_surrond_ind[i];
					*laser_cloud_surrond += *laser_cloud_corner_array[ind];
					*laser_cloud_surrond += *laser_cloud_surf_array[ind];
				}

				sensor_msgs::PointCloud2 laser_cloud_surround_3;
				pcl::toROSMsg(*laser_cloud_surrond, laser_cloud_surround_3);
				laser_cloud_surround_3.header.stamp = ros::Time().fromSec(time_laser_odometry);
				laser_cloud_surround_3.header.frame_id = "/world";
				pub_laser_cloud_surround.publish(laser_cloud_surround_3);
			}

			if (frame_count % 20 == 0)
			{
				PointICloud laser_cloud_map;
				for (int i = 0; i < 4851; i++)
				{
					laser_cloud_map += *laser_cloud_corner_array[i];
					laser_cloud_map += *laser_cloud_surf_array[i];
				}
				sensor_msgs::PointCloud2 laser_cloud_msg;
				pcl::toROSMsg(laser_cloud_map, laser_cloud_msg);
				laser_cloud_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
				laser_cloud_msg.header.frame_id = "/world";
				pub_laser_cloud_map.publish(laser_cloud_msg);
			}

			int laser_cloud_full_res_name = laser_cloud_full_res->points.size();
			for (int i = 0; i < laser_cloud_full_res_name; i++)
			{
				pointAssociateToMap(&laser_cloud_full_res->points[i], &laser_cloud_full_res->points[i]);
			}

			sensor_msgs::PointCloud2 laser_cloud_full_res_3;
			pcl::toROSMsg(*laser_cloud_full_res, laser_cloud_full_res_3);
			laser_cloud_full_res_3.header.stamp = ros::Time().fromSec(time_laser_odometry);
			laser_cloud_full_res_3.header.frame_id = "/world";
			pub_laser_cloud_full_res.publish(laser_cloud_full_res_3);

			printf("mapping pub time %fms \n", t_pub.toc());

			printf("whole mapping time %fms +++++\n", t_whole.toc());
            printf("\n");

			nav_msgs::Odometry odom_aft_mapped;
			odom_aft_mapped.header.frame_id = "/world";
			odom_aft_mapped.child_frame_id = "/aft_mapped";
			odom_aft_mapped.header.stamp = ros::Time().fromSec(time_laser_odometry);
			odom_aft_mapped.pose.pose.orientation.x = q_w_curr.x();
			odom_aft_mapped.pose.pose.orientation.y = q_w_curr.y();
			odom_aft_mapped.pose.pose.orientation.z = q_w_curr.z();
			odom_aft_mapped.pose.pose.orientation.w = q_w_curr.w();
			odom_aft_mapped.pose.pose.position.x = t_w_curr.x();
			odom_aft_mapped.pose.pose.position.y = t_w_curr.y();
			odom_aft_mapped.pose.pose.position.z = t_w_curr.z();
			pub_odom_aft_mapped.publish(odom_aft_mapped);

			geometry_msgs::PoseStamped laser_after_mapped_pose;
			laser_after_mapped_pose.header = odom_aft_mapped.header;
			laser_after_mapped_pose.pose = odom_aft_mapped.pose.pose;
			laser_after_mapped_path.header.stamp = odom_aft_mapped.header.stamp;
			laser_after_mapped_path.header.frame_id = "/world";
			laser_after_mapped_path.poses.push_back(laser_after_mapped_pose);
			pub_laser_after_mapped_path.publish(laser_after_mapped_path);

			static tf::TransformBroadcaster br;
			tf::Transform transform;
			tf::Quaternion q;
			transform.setOrigin(tf::Vector3(t_w_curr(0),
											t_w_curr(1),
											t_w_curr(2)));
			q.setW(q_w_curr.w());
			q.setX(q_w_curr.x());
			q.setY(q_w_curr.y());
			q.setZ(q_w_curr.z());
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, odom_aft_mapped.header.stamp, "/world", "/aft_mapped"));
			frame_count++;

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
		}
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_mapper");
	ros::NodeHandle nh;

	string config_file = argv[1];
	readParameters(config_file);

	// set resolution
	float lineRes = 0.4;
	float planeRes = 0.8;
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
	pub_laser_cloud_full_res = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);

	pub_odom_aft_mapped = nh.advertise<nav_msgs::Odometry>("/laser_map", 100); // raw pose from odometry in the world
	pub_odom_aft_mapped_high_frec = nh.advertise<nav_msgs::Odometry>("/laser_map_high_frec", 100); // optimized pose in the world
	pub_laser_after_mapped_path = nh.advertise<nav_msgs::Path>("/laser_map_path_0", 100);
	for (int i = 0; i < laser_cloud_num; i++)
	{
		laser_cloud_corner_array[i].reset(new PointICloud());
		laser_cloud_surf_array[i].reset(new PointICloud());
	}

	q_ext.resize(NUM_OF_LASER);
	t_ext.resize(NUM_OF_LASER);
	pose_ext.resize(NUM_OF_LASER);

	std::thread mapping_process{process};
	ros::spin();

	return 0;
}
