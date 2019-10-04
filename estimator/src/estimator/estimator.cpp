/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"

using namespace common;

Estimator::Estimator()
{
    ROS_INFO("init begins");
    init_thread_flag_ = false;
    clearState();
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        process_thread_.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    printf("[estimator] clear state\n");
    m_process_.lock();

    b_system_inited_ = false;

    prev_time_ = -1;
    cur_time_ = 0;
    input_cloud_cnt_ = 0;

    td_ = 0;

    solver_flag_ = INITIAL;

    pose_laser_cur_.clear();
    // pose_prev_cur_.clear();
    pose_rlt_.clear();

    qbl_.clear();
    tbl_.clear();
    tdbl_.clear();

    initial_extrinsics_.clearState();

    ini_fixed_local_map_ = false;

    cir_buf_cnt_ = 0;

    Qs_.clear();
    Ts_.clear();
    Header_.clear();
    surf_points_stack_.clear();
    surf_points_stack_size_.clear();
    // corner_points_stack_.clear();
    // corner_points_stack_size_.clear();

    surf_points_local_map_.clear();
    surf_points_local_map_filtered_.clear();
    surf_points_pivot_map_.clear();
    // corner_points_local_map_.clear();
    // corner_points_local_map_filtered_.clear();
    // corner_points_pivot_map_.clear();

    surf_map_features_.clear();
    // corner_map_features_.clear();

    last_marginalization_info_ = nullptr;

    m_process_.unlock();
}

void Estimator::setParameter()
{
    m_process_.lock();

    pose_laser_cur_.resize(NUM_OF_LASER);
    // pose_prev_cur_.resize(NUM_OF_LASER);
    pose_rlt_.resize(NUM_OF_LASER);

    qbl_.resize(NUM_OF_LASER);
    tbl_.resize(NUM_OF_LASER);
    tdbl_.resize(NUM_OF_LASER);
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        qbl_[i] = QBL[i];
        tbl_[i] = TBL[i];
        tdbl_[i] = TDBL[i];
        cout << "Given extrinsic Laser_" << i << ": " << Pose(QBL[i], TBL[i], TDBL[i]) << endl;
    }

    initial_extrinsics_.setParameter();

    Qs_.resize(WINDOW_SIZE + 1);
    Ts_.resize(WINDOW_SIZE + 1);
    Header_.resize(WINDOW_SIZE + 1);
    surf_points_stack_.resize(NUM_OF_LASER);
    surf_points_stack_size_.resize(NUM_OF_LASER);
    // corner_points_stack_.resize(NUM_OF_LASER);
    // corner_points_stack_size_.resize(NUM_OF_LASER);
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        surf_points_stack_[i].resize(WINDOW_SIZE + 1);
        surf_points_stack_size_[i].resize(WINDOW_SIZE + 1);
        // corner_points_stack_[i].resize(WINDOW_SIZE + 1);
        // corner_points_stack_size_[i].resize(WINDOW_SIZE + 1);
    }

    surf_points_local_map_.resize(NUM_OF_LASER);
    surf_points_local_map_filtered_.resize(NUM_OF_LASER);
    surf_points_pivot_map_.resize(NUM_OF_LASER);
    corner_points_local_map_.resize(NUM_OF_LASER);
    corner_points_local_map_filtered_.resize(NUM_OF_LASER);
    corner_points_pivot_map_.resize(NUM_OF_LASER);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !init_thread_flag_)
    {
        init_thread_flag_ = true;
        process_thread_ = std::thread(&Estimator::processMeasurements, this);
    }

    para_pose_ = new double *[OPT_WINDOW_SIZE + 1];
    for (int i = 0; i < OPT_WINDOW_SIZE + 1; i++)
    {
        para_pose_[i] = new double[SIZE_POSE];
    }
    para_ex_pose_ = new double *[NUM_OF_LASER];
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        para_ex_pose_[i] = new double[SIZE_POSE];
    }
    para_td_ = new double[NUM_OF_LASER];

    m_process_.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    m_process_.lock();
    m_process_.unlock();
    if(restart)
    {
        clearState();
        setParameter();
    }
}

void Estimator::inputCloud(const double &t,
    const std::vector<PointCloud> &v_laser_cloud_in)
{
    input_cloud_cnt_++;

    // TODO: to parallize it?
    m_buf_.lock();
    std::vector<cloudFeature> feature_frame;
    TicToc feature_ext_time;
    for (size_t i = 0; i < v_laser_cloud_in.size(); i++)
    {
        printf("[LASER %u]: \n", i);
        feature_frame.push_back(f_extract_.extractCloud(t, v_laser_cloud_in[i]));
    }
    printf("featureExt time: %fms \n", feature_ext_time.toc());
    feature_buf_.push(make_pair(t, feature_frame));
    m_buf_.unlock();

    TicToc process_time;
    processMeasurements();
    ROS_WARN_STREAM("input cloud: " << input_cloud_cnt_ << ", processMea time: " << process_time.toc() << "ms");
}

void Estimator::inputCloud(const double &t,
    const PointCloud &laser_cloud_in0)
{
    input_cloud_cnt_++;
    m_buf_.lock();
    std::vector<cloudFeature> feature_frame;
    TicToc feature_ext_time;
    printf("LASER 0: \n");
    feature_frame.push_back(f_extract_.extractCloud(t, laser_cloud_in0));
    printf("featureExt time: %fms \n", feature_ext_time.toc());
    feature_buf_.push(make_pair(t, feature_frame));
    m_buf_.unlock();

    TicToc process_time;
    // TODO: if MULTIPLE_THREAD
    processMeasurements();
    ROS_WARN_STREAM("frame: " << input_cloud_cnt_ << ", processMea time: " << process_time.toc() << "ms");
}

void Estimator::processMeasurements()
{
    while (1)
    {
        printf("process measurments -----------------------------------\n");
        if (!feature_buf_.empty())
        {
            cur_feature_ = feature_buf_.front();
            cur_time_ = cur_feature_.first + td_;
            assert(cur_feature_.second.size() == NUM_OF_LASER);

            m_buf_.lock();
            feature_buf_.pop();
            m_buf_.unlock();

            m_process_.lock();
            process();

            // -----------------
            // print and publish current result
            printStatistics(*this, 0);
            pubPointCloud(*this, cur_time_);
            pubOdometry(*this, cur_time_);
            // pubKeyPoses(*this, header);
            // pubCameraPose(*this, header);
            // pubKeyframe(*this);
            m_process_.unlock();
        }

        if (!MULTIPLE_THREAD)
            break;

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void Estimator::process()
{
    // -----------------
    if (!b_system_inited_)
    {
        b_system_inited_ = true;
        printf("System initialization finished \n");
        for (int i = 0; i < NUM_OF_LASER; i++)
        {
            pose_rlt_[i] = Pose();
            pose_laser_cur_[i] = Pose();
        }
    } else
    {
        TicToc t_mloam_tracker;
        if (ESTIMATE_EXTRINSIC == 2)
        {
            // feature tracker: estimate the relative transformations of each lidar
            for (int i = 0; i < NUM_OF_LASER; i++)
            {
                printf("[LASER %d]:\n", i);
                cloudFeature &cur_cloud_feature = cur_feature_.second[i];
                cloudFeature &prev_cloud_feature = prev_feature_.second[i];
                pose_rlt_[i] = lidar_tracker_.trackCloud(prev_cloud_feature, cur_cloud_feature, pose_rlt_[i]);
                pose_laser_cur_[i] = pose_laser_cur_[i] * pose_rlt_[i];
                std::cout << "relative transform: " << pose_rlt_[i] << std::endl;
                std::cout << "current transform: " << pose_laser_cur_[i] << std::endl;
            }
            printf("mloam_tracker %fms\n", t_mloam_tracker.toc());

            // initialize extrinsics
            for (int i = 0; i < NUM_OF_LASER; i++) initial_extrinsics_.addPose(pose_rlt_[i], i);
            if (cir_buf_cnt_ == WINDOW_SIZE)
            {
                TicToc t_calib_ext;
                printf("calibrating extrinsic param, rotation movement is needed \n");
                for (int i = 0; i < NUM_OF_LASER; i++)
                {
                    Pose calib_result;
                    if ((!initial_extrinsics_.cov_rot_state_[i]) &&
                        (initial_extrinsics_.calibExRotation(IDX_REF, i, calib_result)))
                    {
                        initial_extrinsics_.setCovRotation(i);
                        ROS_WARN_STREAM("number of pose: " << initial_extrinsics_.frame_cnt_);
                        ROS_WARN_STREAM("initial extrinsic of laser_" << i << ": " << calib_result);
                        qbl_[i] = calib_result.q_;
                        tbl_[i] = calib_result.t_;
                        // tdbl_[i] = calib_result.td_;
                        QBL[i] = calib_result.q_;
                        TBL[i] = calib_result.t_;
                        // TDBL[i] = calib_result.td_;
                    }
                }
                if (initial_extrinsics_.full_cov_rot_state_)
                {
                    ROS_WARN("all initial extrinsic rotation calib success");
                    ESTIMATE_EXTRINSIC = 1;
                    initial_extrinsics_.saveStatistics();
                    // lidar_optimizer_.OptimizeLocalMap();
                }
                printf("whole initialize extrinsics %fms\n", t_calib_ext.toc());
            }
        }
        else if (ESTIMATE_EXTRINSIC != 2)
        {
            cloudFeature &cur_cloud_feature = cur_feature_.second[IDX_REF];
            cloudFeature &prev_cloud_feature = prev_feature_.second[IDX_REF];
            pose_rlt_[IDX_REF] = lidar_tracker_.trackCloud(prev_cloud_feature, cur_cloud_feature, pose_rlt_[IDX_REF]);
            pose_laser_cur_[IDX_REF] = Pose(Qs_[cir_buf_cnt_-1], Ts_[cir_buf_cnt_-1]) * pose_rlt_[IDX_REF];
            std::cout << "relative transform: " << pose_rlt_[IDX_REF] << std::endl;
            std::cout << "current transform: " << pose_laser_cur_[IDX_REF] << std::endl;
            printf("mloam_tracker %fms\n", t_mloam_tracker.toc());
        }
    }

    // get newest pose
    // Qs[cir_buf_cnt_] = Qs.last()
    Qs_[cir_buf_cnt_] = pose_laser_cur_[IDX_REF].q_;
    Ts_[cir_buf_cnt_] = pose_laser_cur_[IDX_REF].t_;

    // get newest point cloud
    Header_[cir_buf_cnt_].stamp = ros::Time(cur_feature_.first);
    PointICloud cloud_downsampled_;
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        // PointICloud &corner_points = cur_feature_.second[i]["corner_points_less_sharp"];
        // f_extract_.down_size_filter_corner_.setInputCloud(boost::make_shared<PointICloud>(corner_points));
        // f_extract_.down_size_filter_corner_.filter(corner_points_stack_downsampled_);
        // corner_points_stack_[i].push(corner_points_stack_downsampled_);
        // corner_points_stack_size_[i].push(corner_points_stack_downsampled_.size());
        PointICloud &surf_points = cur_feature_.second[i]["surf_points_less_flat"];
        f_extract_.down_size_filter_surf_.setInputCloud(boost::make_shared<PointICloud>(surf_points));
        f_extract_.down_size_filter_surf_.filter(cloud_downsampled_);
        surf_points_stack_[i][cir_buf_cnt_] = cloud_downsampled_;
        surf_points_stack_size_[i][cir_buf_cnt_] = cloud_downsampled_.size();
    }
    // printSlideWindow();

    // -----------------
    switch (solver_flag_)
    {
        // INITIAL: multi-LiDAR individual tracking
        case INITIAL:
        {
            printf("[INITIAL]\n");
            slideWindow();
            if (cir_buf_cnt_ < WINDOW_SIZE)
            {
                cir_buf_cnt_++;
                if (cir_buf_cnt_ == WINDOW_SIZE)
                {
                    slideWindow(); // TODO: a bug in the buffer push() function which needs to be fixed
                }
            }
            if ((cir_buf_cnt_ == WINDOW_SIZE) && (ESTIMATE_EXTRINSIC != 2))
            {
                solver_flag_ = NON_LINEAR;
            }
            break;
        }
        // NON_LINEAR: single LiDAR tracking and perform scan-to-map constrains
        case NON_LINEAR:
        {
            // local optimization: optimize the relative LiDAR measurments
            printf("[NON_LINEAR]\n");
            TicToc t_solve;

            optimizeLocalMap();
            slideWindow();

            ROS_DEBUG("solver costs: %fms", t_solve.toc());
            break;
        }
    }
    // printf("size: %d, capacity: %d\n", Qs_.size(), Qs_.capacity());

    // swap features
    prev_time_ = cur_time_;
    prev_feature_.first = prev_time_;
    prev_feature_.second.clear();
    for (size_t i = 0; i < cur_feature_.second.size(); i++)
    {
        cloudFeature tmp_cloud_feature;
        for (auto iter = cur_feature_.second[i].begin(); iter != cur_feature_.second[i].end(); iter++)
        {
            if (iter->first.find("less") != std::string::npos)
                tmp_cloud_feature.insert(make_pair(iter->first, iter->second));
        }
        prev_feature_.second.push_back(tmp_cloud_feature);
    }
    // or
    // prev_feature_.second = cur_feature_.second;
}

void Estimator::buildLocalMap()
{
    TicToc t_build_local_map;
    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    Pose pose_pivot(Qs_[pivot_idx], Ts_[pivot_idx]);
    // -----------------
    // build static local map using fixed poses
    PointICloud surf_points_transformed;
    if (!ini_fixed_local_map_)
    {
        for (int n = 0; n < NUM_OF_LASER; n++)
        {
            PointICloud surf_points_tmp;
            Pose pose_ext = Pose(qbl_[n], tbl_[n]);
            for (size_t i = 0; i <= pivot_idx; i++)
            {
                Pose pose_i(Qs_[i], Ts_[i]);
                Eigen::Affine3d transform_pivot_i;
                transform_pivot_i.matrix() = (pose_pivot.T_ * pose_ext.T_).inverse() * (pose_i.T_ * pose_ext.T_);
                pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_transformed, transform_pivot_i.cast<float>());
                for (auto &p: surf_points_transformed.points) p.intensity = i;

                surf_points_tmp += surf_points_transformed;
                // pcl::transformPointCloud(corner_points_stack_[n][idx], corner_points_transformed, transform_pivot_i);
                // corner_points_tmp[n] += corner_points_transformed;
            }
            surf_points_stack_[n][pivot_idx] = surf_points_tmp;
            // corner_points_stack_[n][pivot_idx] = corner_points_tmp[n];
            printf("Laser_%d: initialize a local map size: %d\n", n, surf_points_stack_[n][pivot_idx].size());
        }
        ini_fixed_local_map_ = true;
    }

    // -----------------
    // build the whole local map using all poses except the newest pose
    for (int n = 0; n < NUM_OF_LASER; n++)
    {
        surf_points_local_map_[n].clear();
        surf_points_local_map_filtered_[n].clear();
        // corner_points_local_map_[n].clear();
        // corner_points_local_map_filtered_[n].clear();
    }
    std::vector<std::vector<Pose> > pose_local;
    pose_local.resize(NUM_OF_LASER);
    for (int n = 0; n < NUM_OF_LASER; n++)
    {
        Pose pose_ext = Pose(qbl_[n], tbl_[n]);
        pose_local[n].resize(WINDOW_SIZE + 1);
        for (int i = 0; i < WINDOW_SIZE + 1; i++)
        {
            Pose pose_i(Qs_[i], Ts_[i]);
            Eigen::Affine3d transform_pivot_i;
            transform_pivot_i.matrix() = (pose_pivot.T_ * pose_ext.T_).inverse() * (pose_i.T_ * pose_ext.T_);
            pose_local[n][i] = Pose(transform_pivot_i.matrix());
            if ((i < pivot_idx) || (i == WINDOW_SIZE)) continue;

            pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_transformed, transform_pivot_i.cast<float>());
            for (auto &p: surf_points_transformed.points) p.intensity = i;
            surf_points_local_map_[n] += surf_points_transformed;
            // pcl::transformPointCloud(corner_points_stack_[n][idx], corner_points_transformed, transform_pivot_i);
            // for (auto &p: corner_points_transformed.points) p.intensity = i;
            // corner_points_local_map_[n] += corner_points_transformed;
        }
        f_extract_.down_size_filter_corner_.setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_[n]));
        f_extract_.down_size_filter_corner_.filter(surf_points_local_map_filtered_[n]);
        // f_extract_.down_size_filter_corner_.setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_));
        // f_extract_.down_size_filter_corner_.filter(corner_points_local_map_filtered_);
        // printf("Laser_%d, filtered local map %d -> %d\n", n, surf_points_local_map_[n].size(), surf_points_local_map_filtered_[n].size());
    }
    printf("build local map: %fms\n", t_build_local_map.toc());
    if (PCL_VIEWER)
    {
        printf("[PlaneNormalVisualizer] update cloud\n");
        if (plane_normal_vis_.init_)
        {
            PointCloud::Ptr point_world_xyz(new PointCloud);
            pcl::copyPointCloud(surf_points_local_map_filtered_[0], *point_world_xyz);
            plane_normal_vis_.UpdateCloud(point_world_xyz, "cloud_all");
            // pcl::io::savePCDFileASCII("/home/jjiao/catkin_ws/src/localization/M-LOAM/log/local_map.pcd", *point_world_xyz);
        }
    }

    // -----------------
    // calculate features and correspondences from p+1 to j
    TicToc t_local_map_extract;
    surf_map_features_.clear();
    surf_map_features_.resize(NUM_OF_LASER);
    for (int n = 0; n < NUM_OF_LASER; n++)
    {
        surf_map_features_[n].resize(WINDOW_SIZE + 1);
        pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_points_local_map(new pcl::KdTreeFLANN<PointI>());
        kdtree_surf_points_local_map->setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_filtered_[n]));
        // pcl::KdTreeFLANN<PointI>::Ptr kdtree_corner_points_local_map(new pcl::KdTreeFLANN<PointI>());
        // kdtree_corner_points_local_map->setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_filtered_[n]));
        for (size_t i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
        {
            f_extract_.extractSurfFromMap(kdtree_surf_points_local_map, surf_points_local_map_filtered_[n],
                surf_points_stack_[n][i], pose_local[n][i], surf_map_features_[n][i]);
            // std::cout << "local pose: " << pose_local[n][i] << std::endl;
            // printf("Laser_%d, %dth window, size of input cloud is %d\n", n, i, surf_points_stack_[n][i].size());

            // f_extract_.extractCornerFromMap(kdtree_corner_points_local_map, corner_points_local_map_filtered_[n],
            //  corner_points_stack_[n][i], pose_local[i], corner_map_features_[n][i]);
            // printf("number of extracted features: %d\n", corner_map_features_[n][i].size());
        }
    }

    // TODO: visualize the correspondences between single frame point cloud and local map
    if (PCL_VIEWER)
    {
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > plane_coeffs;
        PointCloud::Ptr tmp_cloud_sel(new PointCloud); // surf_points_stack_[n][i]
        NormalCloud::Ptr tmp_normals_sel(new NormalCloud); // surf_points_local_map_filtered_[n]
        // printf("feature size: %d\n", surf_map_features_[0][WINDOW_SIZE].size());
        for (auto &f : surf_map_features_[0][WINDOW_SIZE])
        {
            PointI p_ori;
            p_ori.x = f.point_.x();
            p_ori.y = f.point_.y();
            p_ori.z = f.point_.z();
            PointI p_sel;
            f_extract_.pointAssociateToMap(p_ori, p_sel, pose_local[0][WINDOW_SIZE]);
            tmp_cloud_sel->push_back(Point{p_sel.x, p_sel.y, p_sel.z}); // target cloud
            tmp_normals_sel->push_back(Normal{float(f.coeffs_.x()), float(f.coeffs_.y()), float(f.coeffs_.z())}); // reference cloud normal
            // Eigen::Vector4d coeffs_normalized = f.coeffs;
            // double s_normal = coeffs_normalized.head<3>().norm();
            // coeffs_normalized = coeffs_normalized / s_normal;
            // plane_coeffs.push_back(coeffs_normalized);
            // DLOG(INFO) << p_sel.x * f.coeffs.x() + p_sel.y * f.coeffs.y() + p_sel.z * f.coeffs.z() + f.coeffs.w();
        }
        if (plane_normal_vis_.init_)
        {
            plane_normal_vis_.UpdateCloudAndNormals(tmp_cloud_sel, tmp_normals_sel, PCL_VIEWER_NORMAL_RATIO, "cloud1", "normal1");
        }
        // pcl::io::savePCDFileASCII("/home/jjiao/catkin_ws/src/localization/M-LOAM/log/cloud.pcd", *tmp_cloud_sel);
        std::cout << "pose pivot to j: " << pose_local[0][WINDOW_SIZE] << std::endl;
    }
    printf("extract local map: %fms\n", t_local_map_extract.toc());
}

void Estimator::optimizeLocalMap()
{
    TicToc t_prep_solver;
    vector2Double();
    // printParameter();

    // -----------------
    size_t pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    // ceres: set lossfunction and problem
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(0.5);
    loss_function = new ceres::CauchyLoss(1.0);
    ceres::Problem problem;

    // ****************************************************
    // -----------------
    // ceres: add parameter block
    std::vector<double *> para_ids;
    for (size_t i = 0; i < OPT_WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_pose_[i], SIZE_POSE, local_parameterization);
        para_ids.push_back(para_pose_[i]);
    }
    // problem.SetParameterBlockConstant(para_pose_[0]);

    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_ex_pose_[i], SIZE_POSE, local_parameterization);
        para_ids.push_back(para_ex_pose_[i]);
        if (ESTIMATE_EXTRINSIC != 2)
        {
            // printf("estimate extrinsic param");
        } else
        {
            ROS_INFO("extrinsic param are fixed");
            problem.SetParameterBlockConstant(para_ex_pose_[i]);
        }
        if (i == IDX_REF) problem.SetParameterBlockConstant(para_ex_pose_[i]);
    }
    // problem.SetParameterBlockConstant(para_ex_pose_[1]);

    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        problem.AddParameterBlock(&para_td_[i], 1);
        para_ids.push_back(&para_td_[i]);
        if (!ESTIMATE_TD)
        {
            problem.SetParameterBlockConstant(&para_td_[i]);
        }
        if (i == IDX_REF) problem.SetParameterBlockConstant(&para_td_[i]);
    }
    // printf("[ceres] number of parameters: %d\n", problem.NumParameters()); // 6*7+2*7+2=58
    // printf("[ceres] number of parameter vector: %d\n", problem.NumParameterBlocks()); // 8

    // ****************************************************
    // ceres: add marginalization error of previous parameter blocks
    std::vector<ceres::internal::ResidualBlock *> res_ids_marg;
    if ((MARGINALIZATION_FACTOR) && (last_marginalization_info_))
    {
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
        ceres::internal::ResidualBlock *res_id_marg =
            problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks_);
        res_ids_marg.push_back(res_id_marg);
    }

    buildLocalMap();
    // ****************************************************
    // ceres: add residual block within the sliding window
    std::vector<ceres::internal::ResidualBlock *> res_ids_proj;
    if (POINT_PLANE_FACTOR)
    {
        for (int n = 0; n < NUM_OF_LASER - 1; n++)
        // for (int n = 0; n < NUM_OF_LASER - 1; n++)
        {
            for (size_t i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
            // for (size_t i = WINDOW_SIZE; i < WINDOW_SIZE + 1; i++)
            {
                std::vector<PointPlaneFeature> &features_frame = surf_map_features_[n][i];
                // printf("Laser_%d, Win_%d, features: %d\n", n, i, features_frame.size());
                for (auto &feature: features_frame)
                {
                    const double &s = feature.score_;
                    const Eigen::Vector3d &p_data = feature.point_;
                    const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                    ceres::CostFunction *cost_function = LidarPivotPointPlaneFactor::Create(p_data, coeff_ref, s);
                    ceres::internal::ResidualBlock *res_id =
                        problem.AddResidualBlock(cost_function, loss_function, para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[n]);
                    res_ids_proj.push_back(res_id);
                }
            }
        }
    }
    // printf("[ceres] number of residuals: %d\n", problem.NumResiduals());

    if (POINT_EDGE_FACTOR)
    {
    }
    printf("prepare ceres %fms\n", t_prep_solver.toc());

    // -----------------
    // ceres: set options and solve the non-linear equation
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    // options.trust_region_strategy_type = ceres::LEVENBEGR_MARQUARDT;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    options.max_solver_time_in_seconds = SOLVER_TIME;

    printf("*************** Before optimization\n");
    if (EVALUATE_RESIDUAL)
    {
        ///< Bef
        double cost = 0.0;
        ceres::Problem::EvaluateOptions e_option;
        if (POINT_PLANE_FACTOR)
        {
            e_option.parameter_blocks = para_ids;
            e_option.residual_blocks = res_ids_proj;
            problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
            printf("residual bef_proj: %f\n", cost);
        }
        if (MARGINALIZATION_FACTOR)
        {
            if (last_marginalization_info_ && !res_ids_marg.empty())
            {
              e_option.parameter_blocks = para_ids;
              e_option.residual_blocks = res_ids_marg;
              problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
              printf("residual bef_marg: %f\n", cost);
            }
        }
    }

    if (!OPTIMAL_ODOMETRY) return;

    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    // std::cout << summary.FullReport() << std::endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    printf("solver costs: %fms\n", t_solver.toc());

    printf("*************** After optimization\n");
    if (EVALUATE_RESIDUAL)
    {
        ///< Aft
        double cost = 0.0;
        ceres::Problem::EvaluateOptions e_option;
        if (POINT_PLANE_FACTOR)
        {
            e_option.parameter_blocks = para_ids;
            e_option.residual_blocks = res_ids_proj;
            problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
            printf("residual aft_proj: %f\n", cost);
        }
        if (MARGINALIZATION_FACTOR)
        {
            if (last_marginalization_info_ && !res_ids_marg.empty())
            {
              e_option.parameter_blocks = para_ids;
              e_option.residual_blocks = res_ids_marg;
              problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
              printf("residual aft_marg: %f\n", cost);
            }
        }
    }

    double2Vector();
    printParameter();
    // buildLocalMap();

    // ****************************************************
    // ceres: marginalization of current parameter block
    if (MARGINALIZATION_FACTOR)
    {
        TicToc t_whole_marginalization;
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2Double();
        // indicate the marginalized parameter blocks
        if (last_marginalization_info_)
        {
            std::vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++)
            {
                // indicate the dropped pose to calculate the relatd residuals
                if (last_marginalization_parameter_blocks_[i] == para_pose_[0])
                    drop_set.push_back(i);
            }
            // construct marginlization_factor ???
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                last_marginalization_parameter_blocks_, drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        // set marginalized residuals over the marginalized states
        if (POINT_PLANE_FACTOR)
        {
            for (int n = 0; n < NUM_OF_LASER - 1; n++)
            {
                for (size_t i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
                {
                    std::vector<PointPlaneFeature> &features_frame = surf_map_features_[n][i];
                    for (auto &feature: features_frame)
                    {
                        const double &s = feature.score_;
                        const Eigen::Vector3d &p_data = feature.point_;
                        const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                        ceres::CostFunction *cost_function = LidarPivotPointPlaneFactor::Create(p_data, coeff_ref, s);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(cost_function, loss_function,
                            vector<double *>{para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[n]}, std::vector<int>{0});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        //! calculate the residuals and jacobian of all ResidualBlockInfo over the marginalized parameter blocks,
        //! for next iteration, the linearization point is assured and fixed
        //! adjust the memory of H and b to implement the Schur complement
        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG_STREAM("pre marginalization: " << t_pre_margin.toc() << " ms");

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG_STREAM("marginalization: " << t_margin.toc() << " ms");

        //! indicate shared memory of parameter blocks except for the dropped state
        std::unordered_map<long, double *> addr_shift;
        for (size_t i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
        {
            addr_shift[reinterpret_cast<long>(para_pose_[i - pivot_idx])] = para_pose_[i - pivot_idx - 1];
        }
        for (int n = 0; n < NUM_OF_LASER; n++)
        {
            addr_shift[reinterpret_cast<long>(para_ex_pose_[n])] = para_ex_pose_[n];
        }
        for (int n = 0; n < NUM_OF_LASER; n++)
        {
            addr_shift[reinterpret_cast<long>(&para_td_[n])] = &para_td_[n];
        }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info_)
        {
            delete last_marginalization_info_;
        }
        last_marginalization_info_ = marginalization_info;
        last_marginalization_parameter_blocks_ = parameter_blocks;
        ROS_DEBUG_STREAM("whole marginalization costs: " << t_whole_marginalization.toc() << "ms");
    }
}

void Estimator::vector2Double()
{
    size_t pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    for (size_t i = pivot_idx; i < WINDOW_SIZE + 1; i++)
    {
        para_pose_[i - pivot_idx][0] = Ts_[i](0);
        para_pose_[i - pivot_idx][1] = Ts_[i](1);
        para_pose_[i - pivot_idx][2] = Ts_[i](2);
        para_pose_[i - pivot_idx][3] = Qs_[i].x();
        para_pose_[i - pivot_idx][4] = Qs_[i].y();
        para_pose_[i - pivot_idx][5] = Qs_[i].z();
        para_pose_[i - pivot_idx][6] = Qs_[i].w();
    }
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        para_ex_pose_[i][0] = tbl_[i](0);
        para_ex_pose_[i][1] = tbl_[i](1);
        para_ex_pose_[i][2] = tbl_[i](2);
        para_ex_pose_[i][3] = qbl_[i].x();
        para_ex_pose_[i][4] = qbl_[i].y();
        para_ex_pose_[i][5] = qbl_[i].z();
        para_ex_pose_[i][6] = qbl_[i].w();
    }
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        para_td_[i] = tdbl_[i];
    }
}

void Estimator::double2Vector()
{
    size_t pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    for (size_t i = 0; i < OPT_WINDOW_SIZE + 1; i++)
    {
        Ts_[i + pivot_idx] = Eigen::Vector3d(para_pose_[i][0], para_pose_[i][1], para_pose_[i][2]);
        Qs_[i + pivot_idx] = Eigen::Quaterniond(para_pose_[i][6], para_pose_[i][3], para_pose_[i][4], para_pose_[i][5]);
    }
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        tbl_[i] = Eigen::Vector3d(para_ex_pose_[i][0], para_ex_pose_[i][1], para_ex_pose_[i][2]);
        qbl_[i] = Eigen::Quaterniond(para_ex_pose_[i][6], para_ex_pose_[i][3], para_ex_pose_[i][4], para_ex_pose_[i][5]);
    }
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        tdbl_[i] = para_td_[i];
    }
}

void Estimator::printParameter()
{
    printf("print optimized window (p -> j) ************************\n");
    for (size_t i = 0; i < OPT_WINDOW_SIZE + 1; i++)
    {
        std::cout << "Pose: " << " " <<
            para_pose_[i][3] << " " <<
            para_pose_[i][4] << " " <<
            para_pose_[i][5] << " " <<
            para_pose_[i][6] << " " <<
            para_pose_[i][0] << " " <<
            para_pose_[i][1] << " " <<
            para_pose_[i][2] << std::endl;
    }
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        std::cout << "Ext: " << " " <<
            para_ex_pose_[i][3] << " " <<
            para_ex_pose_[i][4] << " " <<
            para_ex_pose_[i][5] << " " <<
            para_ex_pose_[i][6] << " " <<
            para_ex_pose_[i][0] << " " <<
            para_ex_pose_[i][1] << " " <<
            para_ex_pose_[i][2] << std::endl;
    }
    // for (int i = 0; i < NUM_OF_LASER; i++)
    // {
    //     std::cout << "dt: " <<
    //         para_td_[i] << std::endl;
    // }
}

// push new state and measurements in the sliding window
// move the localmap in the pivot frame to the pivot+1 frame, and remove the first point cloud
void Estimator::slideWindow()
{
    TicToc t_solid_window;
    printf("Slide Window, cir_buf_cnt_: %d\n", cir_buf_cnt_);
    if (ini_fixed_local_map_)
    {
        int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
        Pose pose_pivot(Qs_[pivot_idx], Ts_[pivot_idx]);

        PointICloud surf_points_transformed, surf_points_filtered;
        // PointICloud corner_points_transformed, corner_points_filtered;
        int i = pivot_idx + 1;
        Pose pose_i(Qs_[i], Ts_[i]);
        for (int n = 0; n < NUM_OF_LASER; n++)
        {
            surf_points_pivot_map_[n] = surf_points_stack_[n][pivot_idx];

            Pose pose_ext = Pose(qbl_[n], tbl_[n]);
            Eigen::Affine3d transform_i_pivot;
            transform_i_pivot.matrix() = (pose_i.T_ * pose_ext.T_).inverse() * (pose_pivot.T_ * pose_ext.T_);
            pcl::transformPointCloud(surf_points_stack_[n][pivot_idx], surf_points_transformed, transform_i_pivot.cast<float>());

            pcl::PointIndices::Ptr inliers_surf(new pcl::PointIndices());
            for (size_t j = 0; j < surf_points_stack_size_[n][0]; j++) inliers_surf->indices.push_back(j);
            pcl::ExtractIndices<PointI> extract;
            extract.setInputCloud(boost::make_shared<PointICloud>(surf_points_transformed));
            extract.setIndices(inliers_surf);
            extract.setNegative(true);
            extract.filter(surf_points_filtered);
            surf_points_stack_[n][i] = surf_points_filtered + surf_points_stack_[n][i];
        }
    }
    Qs_.push(Qs_[cir_buf_cnt_]);
    Ts_.push(Ts_[cir_buf_cnt_]);
    Header_.push(Header_[cir_buf_cnt_]);
    // std::cout << "pose: " << Pose(Qs_.last(), Ts_.last()) << std::endl;
    for (int n = 0; n < NUM_OF_LASER; n++)
    {
        // printf("Laser: %d, current surf size: %d\n", n, surf_points_stack_size_[n].last());
        // printf("Laser: %d, localmap size: %d\n", n, surf_points_local_map_[n].size());
        surf_points_stack_[n].push(surf_points_stack_[n][cir_buf_cnt_]);
        surf_points_stack_size_[n].push(surf_points_stack_size_[n][cir_buf_cnt_]);
    }
    printf("slide window: %fms\n", t_solid_window.toc());
}

void Estimator::printSlideWindow()
{
    printf("print slide window (0 -> j) ************************\n");
    for (size_t i = 0; i < cir_buf_cnt_ + 1; i++)
    {
        Pose pose(Qs_[i], Ts_[i]);
        std::cout << i << ": " << pose << std::endl;
    }
}

//
