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

int frame_cnt = 0;

double time_laser_cloud_surf_last = 0;
double time_laser_cloud_corner_last = 0;
double time_laser_cloud_full_res = 0;
double time_laser_odometry = 0;
double time_ext = 0;

// thread data buffer
std::queue<sensor_msgs::PointCloud2ConstPtr> surf_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> corner_last_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> full_res_buf;
std::queue<nav_msgs::Odometry::ConstPtr> odometry_buf;
std::queue<mloam_msgs::ExtrinsicsConstPtr> ext_buf;
std::mutex m_buf;

PointICloud::Ptr laser_cloud_surf_last(new PointICloud());
PointICloud::Ptr laser_cloud_corner_last(new PointICloud());
PointICloud::Ptr laser_cloud_surf_last_ds(new PointICloud());
PointICloud::Ptr laser_cloud_corner_last_ds(new PointICloud());
PointICloud::Ptr laser_cloud_full_res(new PointICloud());

PointICovCloud::Ptr laser_cloud_surf_from_map_cov(new PointICovCloud());
PointICovCloud::Ptr laser_cloud_corner_from_map_cov(new PointICovCloud());
PointICovCloud::Ptr laser_cloud_surf_from_map_cov_ds(new PointICovCloud());
PointICovCloud::Ptr laser_cloud_corner_from_map_cov_ds(new PointICovCloud());

std::vector<PointICovCloud> laser_cloud_surf_split_cov;
std::vector<PointICovCloud> laser_cloud_corner_split_cov;

pcl::KdTreeFLANN<PointI>::Ptr kdtree_surrounding_keyframes(new pcl::KdTreeFLANN<PointI>());
pcl::KdTreeFLANN<PointI>::Ptr kdtree_global_map_keyframes(new pcl::KdTreeFLANN<PointI>());
pcl::KdTreeFLANN<PointIWithCov>::Ptr kdtree_surf_from_map(new pcl::KdTreeFLANN<PointIWithCov>());
pcl::KdTreeFLANN<PointIWithCov>::Ptr kdtree_corner_from_map(new pcl::KdTreeFLANN<PointIWithCov>());

bool save_new_keyframe;
PointICloud::Ptr surrounding_keyframes(new PointICloud());
PointICloud::Ptr surrounding_keyframes_ds(new PointICloud());

PointICloud::Ptr global_map_keyframes(new PointICloud());
PointICloud::Ptr global_map_keyframes_ds(new PointICloud());

std::vector<int> surrounding_existing_keyframes_id;
std::vector<PointICovCloud::Ptr> surrounding_surf_cloud_keyframes;
std::vector<PointICovCloud::Ptr> surrounding_corner_cloud_keyframes;
std::vector<PointICovCloud::Ptr> surf_cloud_keyframes_cov;
std::vector<PointICovCloud::Ptr> corner_cloud_keyframes_cov;

// downsampling voxel grid
pcl::VoxelGridCovarianceMLOAM<PointI> down_size_filter_surf;
pcl::VoxelGridCovarianceMLOAM<PointI> down_size_filter_corner;
pcl::VoxelGridCovarianceMLOAM<PointIWithCov> down_size_filter_surf_map_cov;
pcl::VoxelGridCovarianceMLOAM<PointIWithCov> down_size_filter_corner_map_cov;
pcl::VoxelGridCovarianceMLOAM<PointIWithCov> down_size_filter_global_map_cov;

pcl::VoxelGridCovarianceMLOAM<PointI> down_size_filter_surrounding_keyframes;
pcl::VoxelGridCovarianceMLOAM<PointI> down_size_filter_global_map_keyframes;

std::vector<int> point_search_ind;
std::vector<float> point_search_sq_dis;

PointI pose_point_cur, pose_point_prev;
Eigen::Quaterniond pose_ori_cur, pose_ori_prev;
std::vector<std::pair<double, Pose> > pose_keyframes_6d;
PointICloud::Ptr pose_keyframes_3d(new PointICloud());

// wmap_T_curr = wmap_T_odom * wodom_T_curr;
// transformation between odom's world and map's world frame
double para_pose[SIZE_POSE];
Pose pose_wmap_prev, pose_wmap_curr, pose_wmap_wodom, pose_wodom_curr;

// ros
nav_msgs::Path laser_after_mapped_path;

ros::Publisher pub_laser_cloud_surrounding, pub_laser_cloud_map;
ros::Publisher pub_laser_cloud_full_res;
ros::Publisher pub_laser_cloud_surf_last_res, pub_laser_cloud_corner_last_res;
ros::Publisher pub_odom_aft_mapped, pub_odom_aft_mapped_high_frec, pub_laser_after_mapped_path;
ros::Publisher pub_keyframes;

// extrinsics
mloam_msgs::Extrinsics extrinsics;
std::vector<Pose> pose_ext;

std::vector<Eigen::Matrix<double, 1, 6> > d_factor_list;
std::vector<Eigen::Matrix<double, 6, 6> > d_eigvec_list;

Eigen::Matrix<double, 6, 6> mat_P;
Eigen::Matrix<double, 6, 6> cov_mapping;

std::vector<double> cov_mapping_list;

double total_mapping = 0.0;
bool is_degenerate;
bool with_ua_flag;

FeatureExtract f_extract;

pcl::PCDWriter pcd_writer;

std::mutex m_process;

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

    geometry_msgs::PoseStamped laser_after_mapped_pose;
    laser_after_mapped_pose.header = odom_aft_mapped.header;
    laser_after_mapped_pose.pose = odom_aft_mapped.pose.pose;
    laser_after_mapped_path.header = odom_aft_mapped.header;
    laser_after_mapped_path.poses.push_back(laser_after_mapped_pose);
    pub_laser_after_mapped_path.publish(laser_after_mapped_path);
    publishTF(odom_aft_mapped);
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

void extractSurroundingKeyFrames()
{
    if (pose_keyframes_6d.size() == 0) return;

    // update the current point
    pose_point_cur.x = pose_wmap_curr.t_[0];
    pose_point_cur.y = pose_wmap_curr.t_[1];
    pose_point_cur.z = pose_wmap_curr.t_[2];

    surrounding_keyframes->clear();
    // surrounding_keyframes_ds->clear();
    kdtree_surrounding_keyframes->setInputCloud(pose_keyframes_3d);
    kdtree_surrounding_keyframes->radiusSearch(pose_point_cur, (double)SURROUNDING_KF_RADIUS, point_search_ind, point_search_sq_dis, 0);

    for (size_t i = 0; i < point_search_ind.size(); i++)
        surrounding_keyframes->push_back(pose_keyframes_3d->points[point_search_ind[i]]);
    // down_size_filter_surrounding_keyframes.setInputCloud(surrounding_keyframes);
    // down_size_filter_surrounding_keyframes.filter(*surrounding_keyframes_ds);

    for (int i = 0; i < surrounding_existing_keyframes_id.size(); i++) // existing keyframes id
    {
        bool existing_flag = false;
        for (int j = 0; j < surrounding_keyframes->size(); j++) // current surrounding keyframes id
        {
            if (surrounding_existing_keyframes_id[i] == (int)surrounding_keyframes->points[j].intensity)
            {
                existing_flag = true;
                break;
            }
        }
        if (!existing_flag)
        {
            surrounding_existing_keyframes_id.erase(surrounding_existing_keyframes_id.begin() + i);
            surrounding_surf_cloud_keyframes.erase(surrounding_surf_cloud_keyframes.begin() + i);
            surrounding_corner_cloud_keyframes.erase(surrounding_corner_cloud_keyframes.begin() + i);
            i--;
        }
    }

    for (int i = 0; i < surrounding_keyframes->size(); i++)
    {
        bool existing_flag = false;
        for (int j = 0; j < surrounding_existing_keyframes_id.size(); j++)
        {
            if (surrounding_existing_keyframes_id[j] == (int)surrounding_keyframes->points[i].intensity)
            {
                existing_flag = true;
                break;
            }
        }
        if (existing_flag)
        {
            continue;
        }
        else
        {
            int key_ind = (int)surrounding_keyframes->points[i].intensity;
            surrounding_existing_keyframes_id.push_back(key_ind);
            const Pose &pose_local = pose_keyframes_6d[key_ind].second;

            PointICovCloud::Ptr surf_trans(new PointICovCloud());
            cloudUCTAssociateToMap(*surf_cloud_keyframes_cov[key_ind], *surf_trans, pose_local, pose_ext);
            surrounding_surf_cloud_keyframes.push_back(surf_trans);

            PointICovCloud::Ptr corner_trans(new PointICovCloud());
            cloudUCTAssociateToMap(*corner_cloud_keyframes_cov[key_ind], *corner_trans, pose_local, pose_ext);
            surrounding_corner_cloud_keyframes.push_back(corner_trans);
        }
    }

    PointICloud::Ptr surrounding_existing_keyframes(new PointICloud());
    PointICloud::Ptr surrounding_existing_keyframes_ds(new PointICloud());
    for (int i = 0; i < surrounding_existing_keyframes_id.size(); i++)
    {
        int key_ind = surrounding_existing_keyframes_id[i];
        PointI point = pose_keyframes_3d->points[key_ind];
        point.intensity = i;
        surrounding_existing_keyframes->push_back(point);
    }
    down_size_filter_surrounding_keyframes.setInputCloud(surrounding_existing_keyframes);
    down_size_filter_surrounding_keyframes.filter(*surrounding_existing_keyframes_ds);
    for (int i = 0; i < surrounding_existing_keyframes_ds->size(); i++)
    {
        int j = (int)surrounding_existing_keyframes_ds->points[i].intensity;
        *laser_cloud_surf_from_map_cov += *surrounding_surf_cloud_keyframes[j];
        *laser_cloud_corner_from_map_cov += *surrounding_corner_cloud_keyframes[j];
    }

    // for (int i = 0; i < surrounding_existing_keyframes_id.size(); i++)
    // {
    //     *laser_cloud_surf_from_map_cov += *surrounding_surf_cloud_keyframes[i];
    //     *laser_cloud_corner_from_map_cov += *surrounding_corner_cloud_keyframes[i];
    // }

    TicToc t_filter;
    m_process.lock();
    down_size_filter_surf_map_cov.setInputCloud(laser_cloud_surf_from_map_cov);
    down_size_filter_surf_map_cov.filter(*laser_cloud_surf_from_map_cov_ds);
    down_size_filter_corner_map_cov.setInputCloud(laser_cloud_corner_from_map_cov);
    down_size_filter_corner_map_cov.filter(*laser_cloud_corner_from_map_cov_ds);
    m_process.unlock();
    printf("corner/surf: before ds: %lu, %lu; after ds: %lu, %lu\n", 
            laser_cloud_corner_from_map_cov->size(), laser_cloud_surf_from_map_cov->size(),
            laser_cloud_corner_from_map_cov_ds->size(), laser_cloud_surf_from_map_cov_ds->size());
    printf("filter time: %fms\n", t_filter.toc()); // 10ms
}

void downsampleCurrentScan()
{
    laser_cloud_surf_last_ds->clear();
    down_size_filter_surf.setInputCloud(laser_cloud_surf_last);
    down_size_filter_surf.filter(*laser_cloud_surf_last_ds);

    laser_cloud_corner_last_ds->clear();
    down_size_filter_corner.setInputCloud(laser_cloud_corner_last);
    down_size_filter_corner.filter(*laser_cloud_corner_last_ds);
    std::cout << "input surf num: " << laser_cloud_surf_last_ds->size()
              << " corner num: " << laser_cloud_corner_last_ds->size() << std::endl;

    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        laser_cloud_surf_split_cov[n].clear();
        laser_cloud_corner_split_cov[n].clear();
    }
    // propagate the extrinsic uncertainty on points
    for (PointI &point_ori : *laser_cloud_surf_last_ds)
    {
        int idx = int(point_ori.intensity); // indicate the lidar id
        PointI point_sel;
        Eigen::Matrix3d cov_point = Eigen::Matrix3d::Zero();
        if (!with_ua_flag) 
        {
            cov_point = COV_MEASUREMENT; // add extrinsic perturbation
        } else
        {
            pointAssociateToMap(point_ori, point_sel, pose_ext[idx].inverse());
            evalPointUncertainty(point_sel, cov_point, pose_ext[idx]);
            if (cov_point.trace() > TRACE_THRESHOLD_BEFORE_MAPPING) continue;
        }
        PointIWithCov point_cov(point_ori, cov_point.cast<float>());
        laser_cloud_surf_split_cov[idx].push_back(point_cov);
    }
    for (PointI &point_ori : *laser_cloud_corner_last_ds)
    {
        int idx = int(point_ori.intensity); // indicate the lidar id
        PointI point_sel;
        Eigen::Matrix3d cov_point = Eigen::Matrix3d::Zero();
        if (!with_ua_flag) 
        {
            cov_point = COV_MEASUREMENT; // add extrinsic perturbation
        } else
        {
            pointAssociateToMap(point_ori, point_sel, pose_ext[idx].inverse());
            evalPointUncertainty(point_sel, cov_point, pose_ext[idx]);
            if (cov_point.trace() > TRACE_THRESHOLD_BEFORE_MAPPING) continue;
        }
        PointIWithCov point_cov(point_ori, cov_point.cast<float>());
        laser_cloud_corner_split_cov[idx].push_back(point_cov);
    }
}

void scan2MapOptimization()
{
    // step 4: perform scan-to-map optimization
    size_t laser_cloud_surf_from_map_num = laser_cloud_surf_from_map_cov_ds->size();
    size_t laser_cloud_corner_from_map_num = laser_cloud_corner_from_map_cov_ds->size();
    printf("map surf num: %lu, corner num: %lu\n", laser_cloud_surf_from_map_num, laser_cloud_corner_from_map_num);
    if ((laser_cloud_surf_from_map_num > 100) && (laser_cloud_corner_from_map_num > 10))
    {
        TicToc t_opt, t_tree;
        kdtree_surf_from_map->setInputCloud(laser_cloud_surf_from_map_cov_ds);
        kdtree_corner_from_map->setInputCloud(laser_cloud_corner_from_map_cov_ds);
        printf("build tree time %fms\n", t_tree.toc());
        printf("********************************\n");
        for (int iter_cnt = 0; iter_cnt < 3; iter_cnt++)
        {
            ceres::Problem problem;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.max_num_iterations = 15;
            // options.max_solver_time_in_seconds = 0.04;
            // options.num_threads = 2;
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
                                               *laser_cloud_surf_from_map_cov_ds,
                                               laser_cloud_surf_split_cov[n],
                                               pose_wmap_curr,
                                               feature_frame,
                                               n,
                                               n_neigh,
                                               false);
                    map_features.insert(map_features.end(), feature_frame.begin(), feature_frame.end());
                    surf_num += feature_frame.size();
                }
                if (POINT_EDGE_FACTOR)
                {
                    std::vector<PointPlaneFeature> feature_frame;
                    f_extract.matchCornerFromMap(kdtree_corner_from_map,
                                                 *laser_cloud_corner_from_map_cov_ds,
                                                 laser_cloud_corner_split_cov[n],
                                                 pose_wmap_curr,
                                                 feature_frame,
                                                 n,
                                                 n_neigh,
                                                 false);
                    map_features.insert(map_features.end(), feature_frame.begin(), feature_frame.end());
                    corner_num += feature_frame.size();
                }
            }
            printf("matching features time: %fms\n", t_match_features.toc());
            LOG_EVERY_N(INFO, 100) << "matching surf & corner features num: " << surf_num << ", " << corner_num;

            // TODO: testing good feature selection
            std::vector<size_t> sel_feature_idx;
            goodFeatureSelect(para_pose,
                              laser_cloud_surf_split_cov, laser_cloud_corner_split_cov,
                              map_features, map_features.size(),
                              sel_feature_idx, FLAGS_gf_ratio);
            printf("selected features num: %lu(%lu)\n", sel_feature_idx.size(), surf_num + corner_num);

            // if ((frame_cnt % 100 == 0) && (FLAGS_gf_ratio != 1.0))
            //     writeFeature(sel_feature_idx, map_features);

            // test the good feature selection
            // {
            //     std::vector<size_t> sel_feature_idx_test;
            //     goodFeatureSelectTest(para_pose,
            //                           laser_cloud_surf_split_cov, laser_cloud_corner_split_cov,
            //                           map_features, map_features.size(),
            //                           sel_feature_idx_test, FLAGS_gf_ratio);
            // }

            TicToc t_add_constraints;
            CHECK_JACOBIAN = 0;
            for (const size_t &fid : sel_feature_idx)
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
            // printf("add constraints: %fms\n", t_add_constraints.toc());

            // ******************************************************
            // TODO:
            // std::vector<PointPlaneFeature> good_surf_features, good_corner_features;
            // goodFeatureMatching(para_pose,
            //                     kdtree_surf_from_map,
            //                     laser_cloud_surf_split_cov,
            //                     FLAGS_gf_ratio,
            //                     );

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
                LOG_EVERY_N(INFO, 100) << "logdet of H: " << common::logDet(mat_H * 134, true);
                LOG_EVERY_N(INFO, 100) << "pose covariance trace: " << cov_mapping.trace();
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
            if (iter_cnt != 2)
                printf("-------------------------------------\n");
        }
        printf("********************************\n");
        // printf("mapping optimization time: %fms\n", t_opt.toc());
    }
    else
    {
        std::cout << common::YELLOW << "Map surf num is not enough" << common::RESET << std::endl;
    }
}

void saveKeyframeAndInsertGraph()
{
    pose_point_cur.x = pose_wmap_curr.t_[0];
    pose_point_cur.y = pose_wmap_curr.t_[1];
    pose_point_cur.z = pose_wmap_curr.t_[2];
    pose_ori_cur = pose_wmap_curr.q_;

    save_new_keyframe = false;
    if (sqrt((pose_point_cur.x - pose_point_prev.x) * (pose_point_cur.x - pose_point_prev.x)
           + (pose_point_cur.y - pose_point_prev.y) * (pose_point_cur.y - pose_point_prev.y) 
           + (pose_point_cur.z - pose_point_prev.z) * (pose_point_cur.z - pose_point_prev.z)) > DISTANCE_KEYFRAMES ||
        pose_ori_cur.angularDistance(pose_ori_prev) / M_PI * 180 > ORIENTATION_KEYFRAMES)
    {
        save_new_keyframe = true;
    }
    if (!save_new_keyframe && pose_keyframes_6d.size() != 0) return;
    pose_point_prev = pose_point_cur;
    pose_ori_prev = pose_ori_cur;

    // if (cloudKeyPoses3D->points.empty())
    // {
    //     gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]), Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])), priorNoise));
    //     initialEstimate.insert(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
    //                                     Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])));
    //     for (int i = 0; i < 6; ++i)
    //         transformLast[i] = transformTobeMapped[i];
    // } else
    // {
    //     gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
    //                                   Point3(transformLast[5], transformLast[3], transformLast[4]));
    //     gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
    //                                 Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
    //     gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
    //     initialEstimate.insert(cloudKeyPoses3D->size(), Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
    //                                                                  Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
    // }

    PointI pose_3d;
    pose_3d.x = pose_wmap_curr.t_[0];
    pose_3d.y = pose_wmap_curr.t_[1];
    pose_3d.z = pose_wmap_curr.t_[2];
    pose_3d.intensity = pose_keyframes_3d->size();

    m_process.lock();
    pose_keyframes_3d->push_back(pose_3d);
    pose_keyframes_6d.push_back(std::make_pair(time_laser_odometry, pose_wmap_curr));
    m_process.unlock();

    PointICovCloud::Ptr surf_keyframe_cov(new PointICovCloud());
    PointICovCloud::Ptr corner_keyframe_cov(new PointICovCloud());
    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        *surf_keyframe_cov += laser_cloud_surf_split_cov[n];
        *corner_keyframe_cov += laser_cloud_corner_split_cov[n];
    }
    m_process.lock();
    surf_cloud_keyframes_cov.push_back(surf_keyframe_cov);
    corner_cloud_keyframes_cov.push_back(corner_keyframe_cov);
    m_process.unlock();
    printf("current keyframes size: %lu\n", pose_keyframes_3d->size());
}

void pubPointCloud()
{
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
}

void pubOdometry()
{
    // step 5: publish odom
    nav_msgs::Odometry odom_aft_mapped;
    odom_aft_mapped.header.stamp = ros::Time().fromSec(time_laser_odometry);
    odom_aft_mapped.header.frame_id = "/world";
    odom_aft_mapped.child_frame_id = "/aft_mapped";
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

    if (pub_keyframes.getNumSubscribers() != 0)
    {
        m_process.lock();
        sensor_msgs::PointCloud2 keyframes_msg;
        pcl::toROSMsg(*pose_keyframes_3d, keyframes_msg);
        m_process.unlock();
        keyframes_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
        keyframes_msg.header.frame_id = "/world";
        pub_keyframes.publish(keyframes_msg);
    }
}

void pubGlobalMap()
{
    ros::Rate rate(0.2);
    while (ros::ok())
    {
        rate.sleep();
        if (pub_laser_cloud_surrounding.getNumSubscribers() != 0)
        {
            m_process.lock();
            sensor_msgs::PointCloud2 laser_cloud_surround_msg;
            pcl::toROSMsg(*laser_cloud_surf_from_map_cov_ds, laser_cloud_surround_msg);
            m_process.unlock();
            laser_cloud_surround_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
            laser_cloud_surround_msg.header.frame_id = "/world";
            pub_laser_cloud_surrounding.publish(laser_cloud_surround_msg);
            // printf("size of surround map: %d\n", laser_cloud_surrond.size());
        }

        if ((pub_laser_cloud_map.getNumSubscribers() != 0) && (!pose_keyframes_3d->points.empty()))
        {
            global_map_keyframes->clear();
            global_map_keyframes_ds->clear();

            std::vector<int> point_search_ind;
            std::vector<float> point_search_sq_dis;

            kdtree_global_map_keyframes->setInputCloud(pose_keyframes_3d);            
            kdtree_global_map_keyframes->radiusSearch(pose_point_cur, (double)GLOBALMAP_KF_RADIUS, point_search_ind, point_search_sq_dis, 0);

            for (int i = 0; i < point_search_ind.size(); i++)
                global_map_keyframes->points.push_back(pose_keyframes_3d->points[point_search_ind[i]]);
            down_size_filter_global_map_keyframes.setInputCloud(global_map_keyframes);
            down_size_filter_global_map_keyframes.filter(*global_map_keyframes_ds);

            PointICovCloud::Ptr laser_cloud_map(new PointICovCloud());
            PointICovCloud::Ptr  laser_cloud_map_ds(new PointICovCloud());
            for (int i = 0; i < global_map_keyframes_ds->size(); i++)
            {
                int key_ind = (int)global_map_keyframes_ds->points[i].intensity;
                PointICovCloud surf_trans;
                cloudUCTAssociateToMap(*surf_cloud_keyframes_cov[key_ind], surf_trans, pose_keyframes_6d[key_ind].second, pose_ext);
                *laser_cloud_map += surf_trans;

                PointICovCloud corner_trans;
                cloudUCTAssociateToMap(*corner_cloud_keyframes_cov[key_ind], corner_trans, pose_keyframes_6d[key_ind].second, pose_ext);
                *laser_cloud_map += corner_trans;
            }
            down_size_filter_global_map_cov.setInputCloud(laser_cloud_map);
            down_size_filter_global_map_cov.filter(*laser_cloud_map_ds);

            sensor_msgs::PointCloud2 laser_cloud_msg;
            pcl::toROSMsg(*laser_cloud_map_ds, laser_cloud_msg);
            laser_cloud_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
            laser_cloud_msg.header.frame_id = "/world";
            pub_laser_cloud_map.publish(laser_cloud_msg);
        }
    }
}

void saveGlobalMap()
{
    std::cout << common::YELLOW << "Saving keyframe poses & map cloud (corner + surf) tomp/mloam_*.pcd" << common::RESET << std::endl;
    pcd_writer.write("/tmp/mloam_mapping_keyframes.pcd", *pose_keyframes_3d);
    
    PointICovCloud::Ptr laser_cloud_map(new PointICovCloud());
    PointICovCloud::Ptr laser_cloud_surf_map(new PointICovCloud());
    PointICovCloud::Ptr laser_cloud_surf_map_ds(new PointICovCloud());
    PointICovCloud::Ptr laser_cloud_corner_map(new PointICovCloud());
    PointICovCloud::Ptr laser_cloud_corner_map_ds(new PointICovCloud());

    printf("global keyframes num: %lu\n", surf_cloud_keyframes_cov.size());
    for (size_t i = 0; i < surf_cloud_keyframes_cov.size(); i++)
    {
        PointICovCloud surf_trans;
        cloudUCTAssociateToMap(*surf_cloud_keyframes_cov[i], surf_trans, pose_keyframes_6d[i].second, pose_ext);
        *laser_cloud_surf_map += surf_trans;

        PointICovCloud corner_trans;
        cloudUCTAssociateToMap(*corner_cloud_keyframes_cov[i], corner_trans, pose_keyframes_6d[i].second, pose_ext);
        *laser_cloud_corner_map += corner_trans;
    }
    down_size_filter_surf_map_cov.setInputCloud(laser_cloud_surf_map);
    down_size_filter_surf_map_cov.filter(*laser_cloud_surf_map_ds);
    down_size_filter_corner_map_cov.setInputCloud(laser_cloud_corner_map);
    down_size_filter_corner_map_cov.filter(*laser_cloud_corner_map_ds);

    *laser_cloud_map += *laser_cloud_surf_map_ds;
    *laser_cloud_map += *laser_cloud_corner_map_ds;

    pcd_writer.write("/tmp/mloam_mapping_corner_cloud.pcd", *laser_cloud_corner_map_ds);
    pcd_writer.write("/tmp/mloam_mapping_surf_cloud.pcd", *laser_cloud_surf_map_ds);
    pcd_writer.write("/tmp/mloam_mapping_cloud.pcd", *laser_cloud_map);
}

void clearCloud()
{
    laser_cloud_surf_from_map_cov->clear();
    laser_cloud_corner_from_map_cov->clear();
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

			if (std::abs(time_laser_cloud_surf_last - time_laser_cloud_corner_last) > 0.005 ||
				std::abs(time_laser_cloud_surf_last - time_laser_cloud_full_res) > 0.005 ||
				std::abs(time_laser_cloud_surf_last - time_laser_odometry) > 0.005 ||
				std::abs(time_laser_cloud_surf_last - time_ext) > 0.005)
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
			surf_last_buf.pop();

			laser_cloud_corner_last->clear();
			pcl::fromROSMsg(*corner_last_buf.front(), *laser_cloud_corner_last);
			corner_last_buf.pop();

			laser_cloud_full_res->clear();
			pcl::fromROSMsg(*full_res_buf.front(), *laser_cloud_full_res);
			full_res_buf.pop();

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
							pose_ext[n].cov_(i, j) = double(extrinsics.odoms[n].pose.covariance[i * 6 + j]);
				}
			}
			ext_buf.pop();

			while (!surf_last_buf.empty())
			{
				surf_last_buf.pop();
				std::cout << common::GREEN << "drop lidar frame in mapping for real time performance" << common::RESET << std::endl;
			}
			m_buf.unlock();
			
			if (extrinsics.status) continue; // calibration is not finish
			frame_cnt++;
			TicToc t_whole_mapping;

			transformAssociateToMap();

            TicToc t_extract;
            extractSurroundingKeyFrames();
            printf("extract surrounding keyframes: %fms\n", t_extract.toc());

            TicToc t_dscs;
            downsampleCurrentScan();
            printf("downsample current scan time: %fms\n", t_dscs.toc());

            TicToc t_opti;
            scan2MapOptimization();
            printf("optimization time: %fms\n", t_opti.toc());

			transformUpdate();

            TicToc t_skf;
            saveKeyframeAndInsertGraph();
            printf("save keyframes time: %fms\n", t_skf.toc());

            TicToc t_pub;
            pubPointCloud();
            printf("mapping pub time: %fms\n", t_pub.toc());
            LOG_EVERY_N(INFO, 100) << "mapping pub time: " << t_pub.toc() << "ms";

            pubOdometry();

            clearCloud();

            std::cout << common::RED << "frame: " << frame_cnt
                      << ", whole mapping time " << t_whole_mapping.toc() << "ms" << common::RESET << std::endl;
            LOG_EVERY_N(INFO, 100) << "whole mapping time " << t_whole_mapping.toc() << "ms";
            total_mapping += t_whole_mapping.toc();

            // std::cout << "pose_wmap_curr: " << pose_wmap_curr << std::endl;
			printf("\n");
		}
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}
}

void cloudUCTAssociateToMap(const PointICovCloud &cloud_local, PointICovCloud &cloud_global,
                            const Pose &pose_global, const vector<Pose> &pose_ext)
{
    std::vector<Pose> pose_compound(NUM_OF_LASER);
    for (size_t n = 0; n < NUM_OF_LASER; n++)
        compoundPoseWithCov(pose_global, pose_ext[n], pose_compound[n]);

    cloud_global.clear();
    for (const PointIWithCov &point_ori : cloud_local)
    {
        int ind = (int)point_ori.intensity;
        PointIWithCov point_sel, point_cov;
        Eigen::Matrix3d cov_point = Eigen::Matrix3d::Zero();
        if (!with_ua_flag) 
        {
            cov_point = COV_MEASUREMENT;
        } else
        {
            pointAssociateToMap(point_ori, point_sel, pose_ext[ind].inverse());
            evalPointUncertainty(point_sel, cov_point, pose_compound[ind]);
            if (cov_point(0, 0) + cov_point(1, 1) + cov_point(2, 2) > TRACE_THRESHOLD_AFTER_MAPPING) 
                continue;
        }
        pointAssociateToMap(point_ori, point_cov, pose_global);
        updateCov(point_cov, cov_point);
        cloud_global.push_back(point_cov);
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
	for (size_t j = 0; j < mat_E.cols(); j++)
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
 	mat_P = (mat_V_f.transpose()).inverse() * mat_V_p.transpose(); // 6*6

	LOG_EVERY_N(INFO, 100) << "D factor: " << mat_E(0, 0) << ", D vector: " << mat_V_f.col(0).transpose();
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

void sigintHandler(int sig)
{
    printf("press ctrl-c\n");
    if (MLOAM_RESULT_SAVE)
    {
        save_statistics.saveMapStatistics(MLOAM_MAP_PATH,
                                          OUTPUT_FOLDER + "mapping_factor.txt",
                                          OUTPUT_FOLDER + "mapping_d_eigvec.txt",
                                          OUTPUT_FOLDER + "mapping_pose_uncertainty.txt",
                                          laser_after_mapped_path,
                                          d_factor_list,
                                          d_eigvec_list,
                                          cov_mapping_list);
        save_statistics.saveMapTimeStatistics(OUTPUT_FOLDER + "time_mapping.txt", total_mapping, frame_cnt);
    }
    saveGlobalMap();
    ros::shutdown();
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
    OUTPUT_FOLDER = FLAGS_output_path;
	with_ua_flag = FLAGS_with_ua;
    printf("save result (0/1): %d to %s\n", MLOAM_RESULT_SAVE, OUTPUT_FOLDER.c_str());
	printf("uncertainty propagation on (0/1): %d\n", with_ua_flag);
	
	stringstream ss;
	if (with_ua_flag)
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

	ros::Subscriber sub_laser_cloud_full_res = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud", 2, laserCloudFullResHandler);
	ros::Subscriber sub_laser_cloud_surf_last = nh.subscribe<sensor_msgs::PointCloud2>("/surf_points_less_flat", 2, laserCloudSurfLastHandler);
	ros::Subscriber sub_laser_cloud_corner_last = nh.subscribe<sensor_msgs::PointCloud2>("/corner_points_less_sharp", 2, laserCloudCornerLastHandler);
	ros::Subscriber sub_laser_odometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom", 5, laserOdometryHandler);
	ros::Subscriber sub_extrinsic = nh.subscribe<mloam_msgs::Extrinsics>("/extrinsics", 5, extrinsicsHandler);

	pub_laser_cloud_surrounding = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
	pub_laser_cloud_map = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 2);
	pub_laser_cloud_full_res = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_registered", 2);
	pub_laser_cloud_surf_last_res = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_registered", 2);
	pub_laser_cloud_corner_last_res = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_registered", 2);

	pub_odom_aft_mapped = nh.advertise<nav_msgs::Odometry>("/laser_map", 5); // raw pose from odometry in the world
	pub_odom_aft_mapped_high_frec = nh.advertise<nav_msgs::Odometry>("/laser_map_high_frec", 5); // optimized pose in the world
	pub_laser_after_mapped_path = nh.advertise<nav_msgs::Path>("/laser_map_path", 5);
    pub_keyframes = nh.advertise<sensor_msgs::PointCloud2>("/laser_keyframes", 2);

    down_size_filter_surf.setLeafSize(MAP_SURF_RES, MAP_SURF_RES, MAP_SURF_RES);
    down_size_filter_surf.setTraceThreshold(TRACE_THRESHOLD_AFTER_MAPPING);
    down_size_filter_corner.setLeafSize(MAP_CORNER_RES, MAP_CORNER_RES, MAP_CORNER_RES);
    down_size_filter_corner.setTraceThreshold(TRACE_THRESHOLD_AFTER_MAPPING);

    down_size_filter_surf_map_cov.setLeafSize(MAP_SURF_RES, MAP_SURF_RES, MAP_SURF_RES);
    down_size_filter_surf_map_cov.setTraceThreshold(TRACE_THRESHOLD_AFTER_MAPPING);
    down_size_filter_corner_map_cov.setLeafSize(MAP_CORNER_RES, MAP_CORNER_RES, MAP_CORNER_RES);
    down_size_filter_corner_map_cov.setTraceThreshold(TRACE_THRESHOLD_AFTER_MAPPING);
    down_size_filter_surrounding_keyframes.setLeafSize(1.0, 1.0, 1.0);

    down_size_filter_global_map_cov.setLeafSize(MAP_CORNER_RES, MAP_SURF_RES, MAP_SURF_RES);
    down_size_filter_global_map_cov.setTraceThreshold(TRACE_THRESHOLD_AFTER_MAPPING);
    down_size_filter_global_map_keyframes.setLeafSize(1.0, 1.0, 1.0);

	laser_cloud_surf_split_cov.resize(NUM_OF_LASER);
	laser_cloud_corner_split_cov.resize(NUM_OF_LASER);

    pose_ext.resize(NUM_OF_LASER);

    pose_point_prev.x = 0.0;
    pose_point_prev.y = 0.0;
    pose_point_prev.z = 0.0;
    pose_ori_prev.setIdentity();

    pose_point_cur.x = 0.0;
    pose_point_cur.y = 0.0;
    pose_point_cur.z = 0.0;
    pose_ori_cur.setIdentity();

    pose_keyframes_6d.clear();
    pose_keyframes_3d->clear();

    signal(SIGINT, sigintHandler);

    std::thread mapping_process{process};
    std::thread pub_map_process{pubGlobalMap};

    ros::Rate loop_rate(100);
	while (ros::ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
    }

    pub_map_process.join();
    mapping_process.join();
	return 0;
}



