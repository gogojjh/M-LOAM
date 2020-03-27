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

void Estimator::setParameter()
{
    m_process_.lock();

    pose_rlt_.resize(NUM_OF_LASER);
    pose_laser_cur_.resize(NUM_OF_LASER);
    for (auto i = 0; i < NUM_OF_LASER; i++)
    {
        pose_rlt_[i] = Pose();
        pose_laser_cur_[i] = Pose();
    }

    qbl_.resize(NUM_OF_LASER);
    tbl_.resize(NUM_OF_LASER);
    tdbl_.resize(NUM_OF_LASER);
    covbl_.resize(NUM_OF_LASER);
    for (auto i = 0; i < NUM_OF_LASER; i++)
    {
        qbl_[i] = QBL[i];
        tbl_[i] = TBL[i];
        tdbl_[i] = TDBL[i];
        covbl_[i] = COV_EXT[i];
        cout << "Given extrinsic Laser_" << i << ": " << Pose(QBL[i], TBL[i], TDBL[i]) << endl;
    }

    initial_extrinsics_.setParameter();

    Qs_.resize(WINDOW_SIZE + 1);
    Ts_.resize(WINDOW_SIZE + 1);
    Header_.resize(WINDOW_SIZE + 1);
    surf_points_stack_.resize(NUM_OF_LASER);
    surf_points_stack_size_.resize(NUM_OF_LASER);
    corner_points_stack_.resize(NUM_OF_LASER);
    corner_points_stack_size_.resize(NUM_OF_LASER);

    pose_local_.resize(NUM_OF_LASER);
    for (auto i = 0; i < NUM_OF_LASER; i++)
    {
        surf_points_stack_[i].resize(WINDOW_SIZE + 1);
        surf_points_stack_size_[i].resize(WINDOW_SIZE + 1);
        corner_points_stack_[i].resize(WINDOW_SIZE + 1);
        corner_points_stack_size_[i].resize(WINDOW_SIZE + 1);
        pose_local_[i].resize(WINDOW_SIZE + 1);
    }

    surf_points_local_map_.resize(NUM_OF_LASER);
    surf_points_local_map_filtered_.resize(NUM_OF_LASER);
    surf_points_pivot_map_.resize(NUM_OF_LASER);
    corner_points_local_map_.resize(NUM_OF_LASER);
    corner_points_local_map_filtered_.resize(NUM_OF_LASER);
    corner_points_pivot_map_.resize(NUM_OF_LASER);

    cumu_surf_map_features_.resize(NUM_OF_LASER);
    cumu_corner_map_features_.resize(NUM_OF_LASER);

    printf("MULTIPLE_THREAD is %d\n", MULTIPLE_THREAD);
    if (MULTIPLE_THREAD && !init_thread_flag_)
    {
        init_thread_flag_ = true;
        process_thread_ = std::thread(&Estimator::processMeasurements, this);
    }

    para_pose_ = new double *[OPT_WINDOW_SIZE + 1];
    for (auto i = 0; i < OPT_WINDOW_SIZE + 1; i++)
    {
        para_pose_[i] = new double[SIZE_POSE];
    }
    para_ex_pose_ = new double *[NUM_OF_LASER];
    for (auto i = 0; i < NUM_OF_LASER; i++)
    {
        para_ex_pose_[i] = new double[SIZE_POSE];
    }
    para_td_ = new double[NUM_OF_LASER];

    eig_thre_ = std::vector<double>(OPT_WINDOW_SIZE + NUM_OF_LASER + 1, EIG_INITIAL * (NUM_OF_LASER - 1));
    for (auto i = 0; i < NUM_OF_LASER; i++) eig_thre_[OPT_WINDOW_SIZE + i + 1] = 0;
    d_factor_calib_ = std::vector<double>(NUM_OF_LASER, 0);
    cur_eig_calib_ = std::vector<double>(NUM_OF_LASER, 0);
    pose_calib_.resize(NUM_OF_LASER);
    calib_converge_.resize(NUM_OF_LASER, false);

    img_segment_.setScanParam(HORIZON_SCAN, MIN_CLUSTER_SIZE, MIN_LINE_SIZE, SEGMENT_VALID_POINT_NUM, SEGMENT_VALID_LINE_NUM);

    m_process_.unlock();
}

void Estimator::clearState()
{
    printf("[estimator] clear state\n");
    m_process_.lock();

    b_system_inited_ = false;

    prev_time_ = -1;
    cur_time_ = 0;
    frame_cnt_ = 0;

    td_ = 0;

    solver_flag_ = INITIAL;

    pose_rlt_.clear();
    pose_laser_cur_.clear();

    qbl_.clear();
    tbl_.clear();
    tdbl_.clear();
    covbl_.clear();

    initial_extrinsics_.clearState();

    ini_fixed_local_map_ = false;

    cir_buf_cnt_ = 0;

    Qs_.clear();
    Ts_.clear();
    Header_.clear();
    surf_points_stack_.clear();
    surf_points_stack_size_.clear();
    corner_points_stack_.clear();
    corner_points_stack_size_.clear();

    surf_points_local_map_.clear();
    surf_points_local_map_filtered_.clear();
    surf_points_pivot_map_.clear();
    corner_points_local_map_.clear();
    corner_points_local_map_filtered_.clear();
    corner_points_pivot_map_.clear();

    surf_map_features_.clear();
    corner_map_features_.clear();

    cumu_surf_map_features_.clear();
    cumu_corner_map_features_.clear();

    pose_local_.clear();

    last_marginalization_info_ = nullptr;

    eig_thre_.clear();
    d_factor_calib_.clear();
    cur_eig_calib_.clear();
    pose_calib_.clear();
    calib_converge_.clear();

    total_measurement_pre_time_ = 0.0;
    total_opt_odom_time_ = 0.0;

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

void Estimator::inputCloud(const double &t, const std::vector<PointCloud> &v_laser_cloud_in)
{
    assert(v_laser_cloud_in.size() == NUM_OF_LASER);

    TicToc measurement_pre_time;
    std::vector<cloudFeature> feature_frame;
    feature_frame.resize(NUM_OF_LASER);
    printf("size of segmenting cloud: ");
    for (auto i = 0; i < v_laser_cloud_in.size(); i++)
    {
        PointCloud laser_cloud_segment;
        if ((SEGMENT_CLOUD) && (ESTIMATE_EXTRINSIC == 0))
        {
            img_segment_.segmentCloud(v_laser_cloud_in[i], laser_cloud_segment);
            printf("%d ", laser_cloud_segment.size());
            f_extract_.extractCloud(t, laser_cloud_segment, feature_frame[i]);
        } else
        {
            f_extract_.extractCloud(t, v_laser_cloud_in[i], feature_frame[i]);
        }
    }
    printf("\n");
    printf("measurementPre time: %fms (%u*%fms)\n", measurement_pre_time.toc(), v_laser_cloud_in.size(), measurement_pre_time.toc() / v_laser_cloud_in.size());
    total_measurement_pre_time_ += measurement_pre_time.toc();

    m_buf_.lock();
    feature_buf_.push(make_pair(t, feature_frame));
    m_buf_.unlock();
    if (!MULTIPLE_THREAD) processMeasurements();
}

void Estimator::inputCloud(const double &t, const PointCloud &laser_cloud_in)
{
    TicToc measurement_pre_time;
    std::vector<cloudFeature> feature_frame;
    feature_frame.resize(1);
    if ((SEGMENT_CLOUD) && (ESTIMATE_EXTRINSIC == 0))
    {
        PointCloud laser_cloud_segment;
        img_segment_.segmentCloud(laser_cloud_in, laser_cloud_segment);
        f_extract_.extractCloud(t, laser_cloud_segment, feature_frame[0]);
    } else
    {
        f_extract_.extractCloud(t, laser_cloud_in, feature_frame[0]);
    }
    printf("measurementPre time: %fms\n", measurement_pre_time.toc());
    total_measurement_pre_time_ += measurement_pre_time.toc();

    m_buf_.lock();
    feature_buf_.push(make_pair(t, feature_frame));
    m_buf_.unlock();
    if (!MULTIPLE_THREAD) processMeasurements();
}

void Estimator::processMeasurements()
{
    while (1)
    {
        if (!feature_buf_.empty())
        {
            TicToc t_process;
            cur_feature_ = feature_buf_.front();
            cur_time_ = cur_feature_.first + td_;
            assert(cur_feature_.second.size() == NUM_OF_LASER);

            m_buf_.lock();
            feature_buf_.pop();
            m_buf_.unlock();

            m_process_.lock();
            process();

            printStatistics(*this, 0);
            pubOdometry(*this, cur_time_);
            if (frame_cnt_ % SKIP_NUM_ODOM == 0) pubPointCloud(*this, cur_time_); 

            frame_cnt_++;
            ROS_WARN("frame: %d, processMea time: %fms\n", frame_cnt_, t_process.toc());
            m_process_.unlock();
        }
        if (!MULTIPLE_THREAD) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void Estimator::undistortMeasurements()
{
    if (DISTORTION)
    {
        for (auto n = 0; n < NUM_OF_LASER; n++)
        {
            if (ESTIMATE_EXTRINSIC == 2) // initialization
            {
                // for (auto &point : cur_feature_.second[n]["corner_points_sharp"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
                // for (auto &point : cur_feature_.second[n]["surf_points_flat"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
                for (auto &point : cur_feature_.second[n]["corner_points_less_sharp"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
                for (auto &point : cur_feature_.second[n]["surf_points_less_flat"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
                if (frame_cnt_ & SKIP_NUM_ODOM == 0) 
                    for (auto &point : cur_feature_.second[n]["laser_cloud"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
            } else
            if (ESTIMATE_EXTRINSIC == 1) // online calibration
            {
                if (n != IDX_REF) continue;
                // for (auto &point : cur_feature_.second[n]["corner_points_sharp"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
                // for (auto &point : cur_feature_.second[n]["surf_points_flat"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
                for (auto &point : cur_feature_.second[n]["corner_points_less_sharp"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
                for (auto &point : cur_feature_.second[n]["surf_points_less_flat"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
                if (frame_cnt_ & SKIP_NUM_ODOM == 0)
                    for (auto &point : cur_feature_.second[n]["laser_cloud"]) TransformToEnd(point, point, pose_rlt_[n], DISTORTION, SCAN_PERIOD);
            } else
            if (ESTIMATE_EXTRINSIC == 0) // pure odometry
            {
                Pose pose_ext(qbl_[n], tbl_[n]);
                Pose pose_local = pose_ext.inverse() * pose_rlt_[IDX_REF] * pose_ext;
                // for (auto &point : cur_feature_.second[n]["corner_points_sharp"]) TransformToEnd(point, point, pose_local, DISTORTION, SCAN_PERIOD);
                // for (auto &point : cur_feature_.second[n]["surf_points_flat"]) TransformToEnd(point, point, pose_local, DISTORTION, SCAN_PERIOD);
                for (auto &point : cur_feature_.second[n]["corner_points_less_sharp"]) TransformToEnd(point, point, pose_local, DISTORTION, SCAN_PERIOD);
                for (auto &point : cur_feature_.second[n]["surf_points_less_flat"]) TransformToEnd(point, point, pose_local, DISTORTION, SCAN_PERIOD);
                if (frame_cnt_ & SKIP_NUM_ODOM == 0)
                    for (auto &point : cur_feature_.second[n]["laser_cloud"]) TransformToEnd(point, point, pose_local, DISTORTION, SCAN_PERIOD);
            }
        }
    }    
}

void Estimator::process()
{
    if (!b_system_inited_)
    {
        b_system_inited_ = true;
        printf("System initialization finished \n");
    } else
    {
        TicToc t_mloam_tracker;
        // -----------------
        // tracker and initialization
        if (ESTIMATE_EXTRINSIC == 2)
        {
            // feature tracker: estimate the relative transformations of each lidar
            for (auto n = 0; n < NUM_OF_LASER; n++)
            {
                cloudFeature &cur_cloud_feature = cur_feature_.second[n];
                cloudFeature &prev_cloud_feature = prev_feature_.second[n];
                pose_rlt_[n] = lidar_tracker_.trackCloud(prev_cloud_feature, cur_cloud_feature, pose_rlt_[n]);
                pose_laser_cur_[n] = pose_laser_cur_[n] * pose_rlt_[n];
                std::cout << "LASER_" << n << ", pose_rlt: " << pose_rlt_[n] << std::endl;
                // std::cout << "LASER_" << i << ", pose_cur: " << pose_laser_cur_[i] << std::endl;
            }
            printf("lidarTracker: %fms (%d*%fms)\n", t_mloam_tracker.toc(), NUM_OF_LASER, t_mloam_tracker.toc() / NUM_OF_LASER);

            // initialize extrinsics
            if (initial_extrinsics_.addPose(pose_rlt_) && (cir_buf_cnt_ == WINDOW_SIZE))
            {
                TicToc t_calib_ext;
                printf("calibrating extrinsic param, sufficient movement is needed\n");
                for (auto n = 0; n < NUM_OF_LASER; n++)
                {
                    if (initial_extrinsics_.cov_rot_state_[n]) continue;
                    Pose calib_result;
                    if (initial_extrinsics_.calibExRotation(IDX_REF, n, calib_result))
                    {
                        if (initial_extrinsics_.calibExTranslation(IDX_REF, n, calib_result))
                        {
                            std::cout << "Initial extrinsic of laser_" << n << ": " << calib_result << std::endl;
                            qbl_[n] = calib_result.q_;
                            tbl_[n] = calib_result.t_;
                            // tdbl_[n] = calib_result.td_;
                            QBL[n] = calib_result.q_;
                            TBL[n] = calib_result.t_;
                            // TDBL[n] = calib_result.td_;
                        }
                    }
                }
                if ((initial_extrinsics_.full_cov_rot_state_) && (initial_extrinsics_.full_cov_pos_state_))
                {
                    ROS_WARN("All initial extrinsic rotation calib success");
                    ESTIMATE_EXTRINSIC = 1;
                    initial_extrinsics_.saveStatistics();
                }
                printf("initialize extrinsics %fms\n", t_calib_ext.toc());
            }
        }
        // tracker
        else if (ESTIMATE_EXTRINSIC != 2)
        {
            cloudFeature &cur_cloud_feature = cur_feature_.second[IDX_REF];
            cloudFeature &prev_cloud_feature = prev_feature_.second[IDX_REF];
            pose_rlt_[IDX_REF] = lidar_tracker_.trackCloud(prev_cloud_feature, cur_cloud_feature, pose_rlt_[IDX_REF]);
            pose_laser_cur_[IDX_REF] = Pose(Qs_[cir_buf_cnt_-1], Ts_[cir_buf_cnt_-1]) * pose_rlt_[IDX_REF];
            std::cout << "pose_rlt: " << pose_rlt_[IDX_REF] << std::endl;
            // std::cout << "current transform: " << pose_laser_cur_[IDX_REF] << std::endl;
            // Eigen::Vector3d ea = pose_rlt_[IDX_REF].T_.topLeftCorner<3, 3>().eulerAngles(2, 1, 0);
            // printf("relative euler (deg): %f, %f, %f\n", toDeg(ea(0)), toDeg(ea(1)), toDeg(ea(2)));
            printf("lidarTracker %fms\n", t_mloam_tracker.toc());
        }
    }
    undistortMeasurements(); // after tracking, undistort measurements using last frame odometry

    //----------------- update pose and point cloud
    Qs_[cir_buf_cnt_] = pose_laser_cur_[IDX_REF].q_;
    Ts_[cir_buf_cnt_] = pose_laser_cur_[IDX_REF].t_;
    Header_[cir_buf_cnt_].stamp = ros::Time(cur_feature_.first);
    pcl::VoxelGrid<PointI> down_size_filter_corner, down_size_filter_surf;
    down_size_filter_corner.setLeafSize(0.2, 0.2, 0.2);
    down_size_filter_surf.setLeafSize(0.4, 0.4, 0.4);
    PointICloud cloud_downsampled_;
    for (auto n = 0; n < NUM_OF_LASER; n++)
    {
        PointICloud &corner_points = cur_feature_.second[n]["corner_points_less_sharp"];
        down_size_filter_corner.setInputCloud(boost::make_shared<PointICloud>(corner_points));
        down_size_filter_corner.filter(cloud_downsampled_);
        corner_points_stack_[n][cir_buf_cnt_] = cloud_downsampled_;
        corner_points_stack_size_[n][cir_buf_cnt_] = cloud_downsampled_.size();

        PointICloud &surf_points = cur_feature_.second[n]["surf_points_less_flat"];
        down_size_filter_surf.setInputCloud(boost::make_shared<PointICloud>(surf_points));
        down_size_filter_surf.filter(cloud_downsampled_);
        surf_points_stack_[n][cir_buf_cnt_] = cloud_downsampled_;
        surf_points_stack_size_[n][cir_buf_cnt_] = cloud_downsampled_.size();
    }
    // printSlideWindow();

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
            optimizeMap();
            slideWindow();
            if (ESTIMATE_EXTRINSIC) evalCalib();
            break;
        }
    }

    // pass cur_feature to prev_feature
    prev_time_ = cur_time_;
    prev_feature_.first = prev_time_;
    prev_feature_.second.clear();
    prev_feature_.second.resize(NUM_OF_LASER);
    for (auto n = 0; n < NUM_OF_LASER; n++)
    {
        prev_feature_.second[n].insert(make_pair("corner_points_less_sharp", 
            cur_feature_.second[n].find("corner_points_less_sharp")->second));
        prev_feature_.second[n].insert(make_pair("surf_points_less_flat", 
            cur_feature_.second[n].find("surf_points_less_flat")->second));
    }
}

// TODO: optimize_direct_calib
void Estimator::optimizeMap()
{
    TicToc t_opt_map;
    TicToc t_prep_solver;
    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;

    // ****************************************************
    ceres::Problem problem;
    ceres::Solver::Summary summary;
    // ceres: set lossfunction and problem
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(0.1);
    // loss_function = new ceres::CauchyLoss(1.0);
    // ceres: set options and solve the non-linear equation
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.num_threads = 4;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    options.max_solver_time_in_seconds = SOLVER_TIME;

    // ****************************************************
    // ceres: add parameter block
    vector2Double();
    // printParameter();
    // if (!OPTIMAL_ODOMETRY) printParameter();

    std::vector<double *> para_ids;
    std::vector<PoseLocalParameterization *> local_param_ids;
    for (auto i = 0; i < OPT_WINDOW_SIZE + 1; i++)
    {
        PoseLocalParameterization *local_parameterization = new PoseLocalParameterization();
        local_parameterization->setParameter();
        problem.AddParameterBlock(para_pose_[i], SIZE_POSE, local_parameterization);
        local_param_ids.push_back(local_parameterization);
        para_ids.push_back(para_pose_[i]);
    }
    problem.SetParameterBlockConstant(para_pose_[0]);

    for (auto i = 0; i < NUM_OF_LASER; i++)
    {
        PoseLocalParameterization *local_parameterization = new PoseLocalParameterization();
        local_parameterization->setParameter();
        problem.AddParameterBlock(para_ex_pose_[i], SIZE_POSE, local_parameterization);
        local_param_ids.push_back(local_parameterization);
        para_ids.push_back(para_ex_pose_[i]);
        if (ESTIMATE_EXTRINSIC == 0) problem.SetParameterBlockConstant(para_ex_pose_[i]);
    }
    problem.SetParameterBlockConstant(para_ex_pose_[IDX_REF]);

    // for (auto i = 0; i < NUM_OF_LASER; i++)
    // {
    //     problem.AddParameterBlock(&para_td_[i], 1);
    //     para_ids.push_back(&para_td_[i]);
    //     if (!ESTIMATE_TD)
    //     {
    //         problem.SetParameterBlockConstant(&para_td_[i]);
    //     }
    // }
    // problem.SetParameterBlockConstant(&para_td_[IDX_REF]);

    // ****************************************************
    // ceres: add the prior residual into future optimization
    std::vector<ceres::internal::ResidualBlock *> res_ids_marg;
    if ((MARGINALIZATION_FACTOR) && (last_marginalization_info_))
    {
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
        ceres::internal::ResidualBlock *res_id_marg = problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks_);
        res_ids_marg.push_back(res_id_marg);
    }

    // ****************************************************
    // ceres: add residual block within the sliding window
    std::vector<ceres::internal::ResidualBlock *> res_ids_proj;
    if (PRIOR_FACTOR)
    {
        for (auto n = 0; n < NUM_OF_LASER; n++)
        {
            PriorFactor *f = new PriorFactor(tbl_[n], qbl_[n], PRIOR_FACTOR_POS, PRIOR_FACTOR_ROT);
            ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, NULL, para_ex_pose_[n]);
            res_ids_proj.push_back(res_id);
        }
    }

    if (ESTIMATE_EXTRINSIC == 1)
    {
        ROS_WARN("optimization with online calibration");
        buildCalibMap();
        // TODO: using matric representation or ||d||^2 to represent this error, but not easy
        if (POINT_PLANE_FACTOR)
        {
            // CHECK_JACOBIAN = 0;
            for (auto i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
            {
                std::vector<PointPlaneFeature> &features_frame = surf_map_features_[IDX_REF][i];
                // printf("Laser_%d, Win_%d, features: %d\n", n, i, features_frame.size());
                for (std::vector<PointPlaneFeature>::const_iterator iter = features_frame.begin();
                     iter != features_frame.end(); iter++)
                {
                    const Eigen::Vector3d &p_data = iter->point_;
                    const Eigen::Vector4d &coeff_ref = iter->coeffs_;
                    LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, 1.0);
                    ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function,
                        para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[IDX_REF]);
                    res_ids_proj.push_back(res_id);
                    if (CHECK_JACOBIAN)
                    {
                        double **tmp_param = new double *[3];
                        tmp_param[0] = para_pose_[0];
                        tmp_param[1] = para_pose_[i - pivot_idx];
                        tmp_param[2] = para_ex_pose_[IDX_REF];
                        f->check(tmp_param);
                        CHECK_JACOBIAN = 0;
                    }
                }
            }

            for (auto n = 0; n < NUM_OF_LASER; n++) cumu_surf_map_features_[n].push_back(surf_map_features_[n][pivot_idx]);
            if (cumu_surf_map_features_[IDX_REF].size() == N_CUMU_FEATURE)
            {
                ROS_WARN("Start Calibration !");
                for (auto n = 0; n < NUM_OF_LASER; n++)
                {
                    if (n == IDX_REF) continue;
                    for (auto &features_frame: cumu_surf_map_features_[n])
                    {
                        for (std::vector<PointPlaneFeature>::const_iterator iter = features_frame.begin();
                             iter != features_frame.end(); iter++)
                        {
                            const Eigen::Vector3d &p_data = iter->point_;
                            const Eigen::Vector4d &coeff_ref = iter->coeffs_;
                            LidarPivotTargetPlaneNormFactor *f = new LidarPivotTargetPlaneNormFactor(p_data, coeff_ref);
                            ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function, para_ex_pose_[n]);
                            res_ids_proj.push_back(res_id);
                        }
                    }
                }
                if (!MARGINALIZATION_FACTOR)
                {
                    cumu_surf_map_features_.clear();
                    cumu_surf_map_features_.resize(NUM_OF_LASER);
                }
            }
        }

        if (POINT_EDGE_FACTOR)
        {
            for (auto i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
            {
                std::vector<PointPlaneFeature> &features_frame = corner_map_features_[IDX_REF][i];
                // printf("Laser_%d, Win_%d, features: %d\n", n, i, features_frame.size());
                for (std::vector<PointPlaneFeature>::const_iterator iter = features_frame.begin();
                     iter != features_frame.end(); iter++)
                {
                    const Eigen::Vector3d &p_data = iter->point_;
                    const Eigen::Vector4d &coeff_ref = iter->coeffs_;
                    LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, 1.0);
                    ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function,
                        para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[IDX_REF]);
                    res_ids_proj.push_back(res_id);
                }
            }            
            
            for (auto n = 0; n < NUM_OF_LASER; n++) cumu_corner_map_features_[n].push_back(corner_map_features_[n][pivot_idx]);
            if (cumu_corner_map_features_[IDX_REF].size() == N_CUMU_FEATURE)
            {
                for (auto n = 0; n < NUM_OF_LASER; n++)
                {
                    if (n == IDX_REF) continue;
                    for (auto &features_frame: cumu_corner_map_features_[n])
                    {
                        for (std::vector<PointPlaneFeature>::const_iterator iter = features_frame.begin();
                             iter != features_frame.end(); iter++)
                        {
                            const Eigen::Vector3d &p_data = iter->point_;
                            const Eigen::Vector4d &coeff_ref = iter->coeffs_;
                            LidarPivotTargetPlaneNormFactor *f = new LidarPivotTargetPlaneNormFactor(p_data, coeff_ref);
                            ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function, para_ex_pose_[n]);
                            res_ids_proj.push_back(res_id);
                        }
                    }
                }
                if (!MARGINALIZATION_FACTOR)
                {
                    cumu_corner_map_features_.clear();
                    cumu_corner_map_features_.resize(NUM_OF_LASER);
                }
            }
        }
    }
    else if (ESTIMATE_EXTRINSIC == 0)
    {
        ROS_WARN("optimization with pure odometry");
        buildLocalMap();
        if (POINT_PLANE_FACTOR)
        {
            for (auto n = 0; n < NUM_OF_LASER; n++)
            {
                for (auto i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
                {
                    std::vector<PointPlaneFeature> &features_frame = surf_map_features_[n][i];
                    // printf("Laser_%d, Win_%d, features: %d\n", n, i, features_frame.size()); // 500-1000
                    for (std::vector<PointPlaneFeature>::const_iterator iter = features_frame.begin();
                         iter != features_frame.end(); iter++)
                    {
                        const Eigen::Vector3d &p_data = iter->point_;
                        const Eigen::Vector4d &coeff_ref = iter->coeffs_;
                        LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, 1.0);
                        ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function,
                            para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[n]);
                        res_ids_proj.push_back(res_id);
                    }
                }
            }
        }
    }
    // *******************************
    // ROS_WARN("Before optimization");
    if (EVALUATE_RESIDUAL) evalResidual(problem, local_param_ids, para_ids, res_ids_proj, last_marginalization_info_, res_ids_marg, true);
    printf("prepare ceres %fms\n", t_prep_solver.toc()); // cost time

    TicToc t_ceres_solver;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    // std::cout << summary.FullReport() << std::endl;
    printf("ceres solver costs: %fms\n", t_ceres_solver.toc());

    double2Vector();

    // ****************************************************
    // ceres: marginalization of current parameter block
    // prepare all the residuals, jacobians, and dropped parameter blocks to construct marginalization prior 
    if (MARGINALIZATION_FACTOR)
    {
        TicToc t_whole_marginalization;
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2Double();
        // indicate the prior error
        if (last_marginalization_info_)
        {
            std::vector<int> drop_set;
            for (auto i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++)
            {
                // indicate the dropped pose to calculate the related residuals
                if (last_marginalization_parameter_blocks_[i] == para_pose_[0]) drop_set.push_back(i);
            }
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                last_marginalization_parameter_blocks_, drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (PRIOR_FACTOR)
        {
            for (auto n = 0; n < NUM_OF_LASER; n++)
            {
                PriorFactor *f = new PriorFactor(tbl_[n], qbl_[n], PRIOR_FACTOR_POS, PRIOR_FACTOR_ROT);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, NULL,
                    std::vector<double *>{para_ex_pose_[n]}, std::vector<int>{});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        if (ESTIMATE_EXTRINSIC == 1)
        {
            if (POINT_PLANE_FACTOR)
            {
                for (auto i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
                {
                    std::vector<PointPlaneFeature> &features_frame = surf_map_features_[IDX_REF][i];
                    for (auto &feature: features_frame)
                    {
                        const Eigen::Vector3d &p_data = feature.point_;
                        const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                        LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, 1.0);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                            std::vector<double *>{para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[IDX_REF]}, std::vector<int>{0});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }

                if (cumu_surf_map_features_[IDX_REF].size() == N_CUMU_FEATURE)
                {
                    for (auto n = 0; n < NUM_OF_LASER; n++)
                    {
                        // if (n == IDX_REF) continue;
                        for (auto &features_frame: cumu_surf_map_features_[n])
                        {
                            for (auto &feature: features_frame)
                            {
                                const Eigen::Vector3d &p_data = feature.point_;
                                const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                                LidarPivotTargetPlaneNormFactor *f = new LidarPivotTargetPlaneNormFactor(p_data, coeff_ref);
                                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                    std::vector<double *>{para_ex_pose_[n]}, std::vector<int>{});
                                marginalization_info->addResidualBlockInfo(residual_block_info);
                            }
                        }
                    }
                    cumu_surf_map_features_.clear();
                    cumu_surf_map_features_.resize(NUM_OF_LASER);
                }
            }

            if (POINT_EDGE_FACTOR)
            {
                for (auto i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
                {
                    std::vector<PointPlaneFeature> &features_frame = corner_map_features_[IDX_REF][i];
                    for (auto &feature: features_frame)
                    {
                        const Eigen::Vector3d &p_data = feature.point_;
                        const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                        LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, 1.0);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                            std::vector<double *>{para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[IDX_REF]}, std::vector<int>{0});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }                

                if (cumu_corner_map_features_[IDX_REF].size() == N_CUMU_FEATURE)
                {
                    for (auto n = 0; n < NUM_OF_LASER; n++)
                    {
                        // if (n == IDX_REF) continue;
                        for (auto &features_frame: cumu_corner_map_features_[n])
                        {
                            for (auto &feature: features_frame)
                            {
                                const Eigen::Vector3d &p_data = feature.point_;
                                const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                                LidarPivotTargetPlaneNormFactor *f = new LidarPivotTargetPlaneNormFactor(p_data, coeff_ref);
                                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                    std::vector<double *>{para_ex_pose_[n]}, std::vector<int>{});
                                marginalization_info->addResidualBlockInfo(residual_block_info);
                            }
                        }
                    }
                    cumu_corner_map_features_.clear();
                    cumu_corner_map_features_.resize(NUM_OF_LASER);
                }
            }
        }
        else if (ESTIMATE_EXTRINSIC == 0)
        {
            if (POINT_PLANE_FACTOR)
            {
                // std::vector<LidarPivotPlaneNormFactor *> factor_buf;
                for (auto n = 0; n < NUM_OF_LASER; n++)
                {
                    for (auto i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
                    {
                        std::vector<PointPlaneFeature> &features_frame = surf_map_features_[n][i];
                        for (auto &feature: features_frame)
                        {
                            const Eigen::Vector3d &p_data = feature.point_;
                            const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                            LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, 1.0);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                vector<double *>{para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[n]}, std::vector<int>{0});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        //! calculate the residuals and jacobian of all ResidualBlockInfo over the marginalized parameter blocks,
        //! for next iteration, the linearization poauto is assured and fixed
        //! adjust the memory of H and b to implement the Schur complement
        TicToc t_pre_margin;
        marginalization_info->preMarginalize(); // add parameter block given residual info
        // printf("pre marginalization: %fms\n", t_pre_margin.toc());

        TicToc t_margin;
        // marginalize some states and keep the remaining states with prior residuals
        marginalization_info->marginalize(); // compute linear residuals and jacobian
        // printf("marginalization: %fms\n", t_margin.toc());

        //! indicate shared memory of parameter blocks except for the dropped state
        std::unordered_map<long, double *> addr_shift;
        for (auto i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
        {
            addr_shift[reinterpret_cast<long>(para_pose_[i - pivot_idx])] = para_pose_[i - pivot_idx - 1];
        }
        for (auto n = 0; n < NUM_OF_LASER; n++)
        {
            addr_shift[reinterpret_cast<long>(para_ex_pose_[n])] = para_ex_pose_[n];
        }
        // for (auto n = 0; n < NUM_OF_LASER; n++)
        // {
        //     addr_shift[reinterpret_cast<long>(&para_td_[n])] = &para_td_[n];
        // }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info_)
        {
            delete last_marginalization_info_;
        }
        last_marginalization_info_ = marginalization_info;
        last_marginalization_parameter_blocks_ = parameter_blocks; // save parameter_blocks at the last optimization
        printf("whole marginalization costs: %fms\n", t_whole_marginalization.toc());
    }
    total_opt_odom_time_ += t_opt_map.toc();
}

/****************************************************************************************/
// TODO: the calibration refinement should be use to refine with pivot map
void Estimator::buildCalibMap()
{
    TicToc t_build_map;
    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    Pose pose_pivot(Qs_[pivot_idx], Ts_[pivot_idx]);

    // -----------------
    // build the whole local map using all poses except the newest pose
    for (auto n = 0; n < NUM_OF_LASER; n++)
    {
        surf_points_local_map_[n].clear(); surf_points_local_map_filtered_[n].clear();
        corner_points_local_map_[n].clear(); corner_points_local_map_filtered_[n].clear();
    }
    for (auto n = 0; n < NUM_OF_LASER; n++)
    {
        Pose pose_ext = Pose(qbl_[n], tbl_[n]);
        for (auto i = 0; i < WINDOW_SIZE + 1; i++)
        {
            Pose pose_i(Qs_[i], Ts_[i]);
            pose_local_[n][i] = Pose(pose_pivot.T_.inverse() * pose_i.T_ * pose_ext.T_);
            if (i == WINDOW_SIZE) continue;
            if (n == IDX_REF) // localmap of reference
            {
                PointICloud surf_points_trans, corner_points_trans;
                pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_trans, pose_local_[n][i].T_.cast<float>());
                for (auto &p: surf_points_trans.points) p.intensity = i;
                surf_points_local_map_[n] += surf_points_trans;

                pcl::transformPointCloud(corner_points_stack_[n][i], corner_points_trans, pose_local_[n][i].T_.cast<float>());
                for (auto &p: corner_points_trans.points) p.intensity = i;
                corner_points_local_map_[n] += corner_points_trans;
            }
        }

        pcl::VoxelGrid<PointI> down_size_filter;
        if (n == IDX_REF)
        {
            down_size_filter.setLeafSize(0.4, 0.4, 0.4);
            down_size_filter.setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_[IDX_REF]));
            down_size_filter.filter(surf_points_local_map_filtered_[IDX_REF]);
            down_size_filter.setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_[IDX_REF]));
            down_size_filter.filter(corner_points_local_map_filtered_[IDX_REF]);
        } 
        // else
        // {
        //     down_size_filter.setLeafSize(0.3, 0.3, 0.3);
        //     down_size_filter.setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_[IDX_REF]));
        //     down_size_filter.filter(surf_points_local_map_filtered_[n]); 
        //     down_size_filter.setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_[IDX_REF]));
        //     down_size_filter.filter(corner_points_local_map_filtered_[n]);
        // }
    }

    // -----------------
    // calculate features and correspondences from p+1 to j
    TicToc t_extract_map;
    surf_map_features_.clear(); surf_map_features_.resize(NUM_OF_LASER);
    corner_map_features_.clear(); corner_map_features_.resize(NUM_OF_LASER);
    
    pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_points_local_map(new pcl::KdTreeFLANN<PointI>());
    kdtree_surf_points_local_map->setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_filtered_[IDX_REF]));
    pcl::KdTreeFLANN<PointI>::Ptr kdtree_corner_points_local_map(new pcl::KdTreeFLANN<PointI>());
    kdtree_corner_points_local_map->setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_filtered_[IDX_REF]));
    for (auto n = 0; n < NUM_OF_LASER; n++)
    {
        surf_map_features_[n].resize(WINDOW_SIZE + 1);
        corner_map_features_[n].resize(WINDOW_SIZE + 1);
        if (calib_converge_[n]) continue;
        for (auto i = pivot_idx; i < WINDOW_SIZE + 1; i++)
        {
            if (((n == IDX_REF) && (i == pivot_idx)) || ((n != IDX_REF) && (i != pivot_idx))) continue;
            int n_neigh = (n == IDX_REF ? 5:10);
            f_extract_.matchSurfFromMap(kdtree_surf_points_local_map, surf_points_local_map_filtered_[IDX_REF],
                surf_points_stack_[n][i], pose_local_[n][i], surf_map_features_[n][i], n_neigh, true);
            f_extract_.matchCornerFromMap(kdtree_corner_points_local_map, corner_points_local_map_filtered_[IDX_REF],
                corner_points_stack_[n][i], pose_local_[n][i], corner_map_features_[n][i], n_neigh, true);
            // std::vector<PointPlaneFeature> tmp_surf_map_features, tmp_corner_map_features;
            // f_extract_.matchSurfFromMap(kdtree_surf_points_local_map, surf_points_local_map_filtered_[n],
            //     surf_points_stack_[n][i], pose_local_[n][i], tmp_surf_map_features, n_neigh, true);
            // f_extract_.matchCornerFromMap(kdtree_corner_points_local_map, corner_points_local_map_filtered_[n],
            //     corner_points_stack_[n][i], pose_local_[n][i], tmp_corner_map_features, n_neigh, true);
            // std::copy(tmp_surf_map_features.begin(), tmp_surf_map_features.end(), std::back_inserter(surf_map_features_[n][i]));
            // std::copy(tmp_corner_map_features.begin(), tmp_corner_map_features.end(), std::back_inserter(corner_map_features_[n][i]));
        }

    }
    printf("build map (extract map): %f (%f)ms\n", t_build_map.toc(), t_extract_map.toc());
    if (PCL_VIEWER) visualizePCL();
}

void Estimator::buildLocalMap()
{
    TicToc t_build_map;
    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    Pose pose_pivot(Qs_[pivot_idx], Ts_[pivot_idx]);

    // -----------------
    // build the whole local map using all poses except the newest pose
    for (auto n = 0; n < NUM_OF_LASER; n++)
    {
        surf_points_local_map_[n].clear();
        surf_points_local_map_filtered_[n].clear();
        // corner_points_local_map_[n].clear();
        // corner_points_local_map_filtered_[n].clear();
    }
    for (auto n = 0; n < NUM_OF_LASER; n++)
    {
        Pose pose_ext = Pose(qbl_[n], tbl_[n]);
        for (auto i = 0; i < WINDOW_SIZE + 1; i++)
        {
            Pose pose_i(Qs_[i], Ts_[i]);
            pose_local_[n][i] = Pose(pose_pivot.T_.inverse() * pose_i.T_ * pose_ext.T_);
            if (i == WINDOW_SIZE) continue;
            PointICloud surf_points_trans, corner_points_trans;
            pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_trans, pose_local_[n][i].T_.cast<float>());
            for (auto &p: surf_points_trans.points) p.intensity = i;
            surf_points_local_map_[n] += surf_points_trans;
            // pcl::transformPointCloud(corner_points_stack_[n][i], corner_points_trans, pose_local_[n][i].T_.cast<float>());
            // for (auto &p: corner_points_trans.points) p.intensity = i;
            // corner_points_local_map_[n] += corner_points_trans;
        }
        pcl::VoxelGrid<PointI> down_size_filter;
        down_size_filter.setLeafSize(0.4, 0.4, 0.4);
        down_size_filter.setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_[n]));
        down_size_filter.filter(surf_points_local_map_filtered_[n]);
        // down_size_filter.setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_[n]));
        // down_size_filter.filter(corner_points_local_map_filtered_[n]);
    }

    // -----------------
    // calculate features and correspondences from p+1 to j
    surf_map_features_.clear(); surf_map_features_.resize(NUM_OF_LASER);
    for (auto n = 0; n < NUM_OF_LASER; n++)
    {
        surf_map_features_[n].resize(WINDOW_SIZE + 1);
        pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_points_local_map(new pcl::KdTreeFLANN<PointI>());
        kdtree_surf_points_local_map->setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_filtered_[n]));
        // corner_map_features_[n].resize(WINDOW_SIZE + 1);
        // pcl::KdTreeFLANN<PointI>::Ptr kdtree_corner_points_local_map(new pcl::KdTreeFLANN<PointI>());
        // kdtree_corner_points_local_map->setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_filtered_[n]));
        auto n_neigh = 5;
        for (auto i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
        {
            f_extract_.matchSurfFromMap(kdtree_surf_points_local_map, surf_points_local_map_filtered_[n],
                surf_points_stack_[n][i], pose_local_[n][i], surf_map_features_[n][i], n_neigh, true);
            // std::vector<PointPlaneFeature> tmp_map_features;
            // f_extract_.matchSurfFromMap(kdtree_surf_points_local_map, surf_points_local_map_filtered_[n],
            //     surf_points_stack_[n][i], pose_local_[n][i], tmp_map_features, n_neigh, true);
            // f_extract_.extractCornerFromMap(kdtree_corner_points_local_map, corner_points_local_map_filtered_[n],
                //     corner_points_stack_[n][i], pose_local_[n][i], corner_map_features_[n][i], n_neigh, true);
            // std::copy(tmp_map_features.begin(), tmp_map_features.end(), std::back_inserter(surf_map_features_[n][i]));
        }
    }
    printf("build map: %fms\n", t_build_map.toc());
    if (PCL_VIEWER) visualizePCL();
}

// push new state and measurements in the sliding window
// move the localmap in the pivot frame to the pivot+1 frame, and remove the first point cloud
void Estimator::slideWindow()
{
    TicToc t_solid_window;
    printf("sliding window with cir_buf_cnt_: %d\n", cir_buf_cnt_);
    Qs_.push(Qs_[cir_buf_cnt_]);
    Ts_.push(Ts_[cir_buf_cnt_]);
    Header_.push(Header_[cir_buf_cnt_]);
    for (auto n = 0; n < NUM_OF_LASER; n++)
    {
        surf_points_stack_[n].push(surf_points_stack_[n][cir_buf_cnt_]);
        surf_points_stack_size_[n].push(surf_points_stack_size_[n][cir_buf_cnt_]);
        corner_points_stack_[n].push(corner_points_stack_[n][cir_buf_cnt_]);
        corner_points_stack_size_[n].push(corner_points_stack_size_[n][cir_buf_cnt_]);
    }
    // printf("slide window: %fms\n", t_solid_window.toc());
}

void Estimator::vector2Double()
{
    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    for (auto i = pivot_idx; i < WINDOW_SIZE + 1; i++)
    {
        para_pose_[i - pivot_idx][0] = Ts_[i](0);
        para_pose_[i - pivot_idx][1] = Ts_[i](1);
        para_pose_[i - pivot_idx][2] = Ts_[i](2);
        para_pose_[i - pivot_idx][3] = Qs_[i].x();
        para_pose_[i - pivot_idx][4] = Qs_[i].y();
        para_pose_[i - pivot_idx][5] = Qs_[i].z();
        para_pose_[i - pivot_idx][6] = Qs_[i].w();
    }
    for (auto i = 0; i < NUM_OF_LASER; i++)
    {
        para_ex_pose_[i][0] = tbl_[i](0);
        para_ex_pose_[i][1] = tbl_[i](1);
        para_ex_pose_[i][2] = tbl_[i](2);
        para_ex_pose_[i][3] = qbl_[i].x();
        para_ex_pose_[i][4] = qbl_[i].y();
        para_ex_pose_[i][5] = qbl_[i].z();
        para_ex_pose_[i][6] = qbl_[i].w();
    }
    // for (auto i = 0; i < NUM_OF_LASER; i++)
    // {
    //     para_td_[i] = tdbl_[i];
    // }
}

void Estimator::double2Vector()
{
    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    for (auto i = 0; i < OPT_WINDOW_SIZE + 1; i++)
    {
        Ts_[i + pivot_idx] = Eigen::Vector3d(para_pose_[i][0], para_pose_[i][1], para_pose_[i][2]);
        Qs_[i + pivot_idx] = Eigen::Quaterniond(para_pose_[i][6], para_pose_[i][3], para_pose_[i][4], para_pose_[i][5]);
    }
    for (auto i = 0; i < NUM_OF_LASER; i++)
    {
        tbl_[i] = Eigen::Vector3d(para_ex_pose_[i][0], para_ex_pose_[i][1], para_ex_pose_[i][2]);
        qbl_[i] = Eigen::Quaterniond(para_ex_pose_[i][6], para_ex_pose_[i][3], para_ex_pose_[i][4], para_ex_pose_[i][5]);
    }
    // for (auto i = 0; i < NUM_OF_LASER; i++)
    // {
    //     tdbl_[i] = para_td_[i];
    // }
}

void Estimator::evalResidual(ceres::Problem &problem,
    std::vector<PoseLocalParameterization *> &local_param_ids,
    const std::vector<double *> &para_ids,
    const std::vector<ceres::internal::ResidualBlock *> &res_ids_proj,
	const MarginalizationInfo *last_marginalization_info_,
    const std::vector<ceres::internal::ResidualBlock *> &res_ids_marg,
    const bool b_eval_degenracy)
{
	double cost;
    ceres::CRSMatrix jaco;
    ceres::Problem::EvaluateOptions e_option;
	if ((PRIOR_FACTOR) || (POINT_PLANE_FACTOR) || (POINT_EDGE_FACTOR))
	{
		e_option.parameter_blocks = para_ids;
		e_option.residual_blocks = res_ids_proj;
        problem.Evaluate(e_option, &cost, nullptr, nullptr, &jaco);
        // printf("cost proj: %f\n", cost);
        if (b_eval_degenracy) evalDegenracy(local_param_ids, jaco);
	}
	if (MARGINALIZATION_FACTOR)
	{
		if (last_marginalization_info_ && !res_ids_marg.empty())
		{
			e_option.parameter_blocks = para_ids;
			e_option.residual_blocks = res_ids_marg;
            problem.Evaluate(e_option, &cost, nullptr, nullptr, &jaco);
            // printf("cost marg: %f\n", cost);
		}
	}
}

// A^TA is not only symmetric and invertiable: https://math.stackexchange.com/questions/2352684/when-is-a-symmetric-matrix-invertible
void Estimator::evalDegenracy(std::vector<PoseLocalParameterization *> &local_param_ids, const ceres::CRSMatrix &jaco)
{
    // printf("jacob: %d constraints, %d parameters (%d pose_param, %d ext_param)\n",
    //     jaco.num_rows, jaco.num_cols, 6*(OPT_WINDOW_SIZE + 1), 6*NUM_OF_LASER); // 58, feature_size(2700) x 50
    if (jaco.num_rows == 0) return;
    TicToc t_eval_degenracy;
    Eigen::MatrixXd mat_J;
    CRSMatrix2EigenMatrix(jaco, mat_J);
    Eigen::MatrixXd mat_Jt = mat_J.transpose(); // A^T
    Eigen::MatrixXd mat_JtJ = mat_Jt * mat_J; // A^TA 48*48, rewrite as a 6*6 diagonal matrix 
    bool b_vis = false; // to verify the structure of A^T*A
    if (b_vis)
    {
        printf("visualize the structure of H(J^T*J)\n");
        for (auto i = 0; i < mat_JtJ.rows(); i++)
        {
            for (auto j = 0; j < mat_JtJ.cols(); j++)
            {
                if (mat_JtJ(i, j) == 0) std::cout << "0 ";
                                    else std::cout << "1 ";
            }
            std::cout << std::endl;
        }
    }

    double eig_thre; // the larger, the better (with more constraints)
    d_factor_calib_ = std::vector<double>(NUM_OF_LASER, 0);
    // cur_eig_calib_ = std::vector<double>(NUM_OF_LASER, 0);
    for (auto i = 0; i < local_param_ids.size(); i++)
    {
        Eigen::Matrix<double, 6, 6> mat_H = mat_JtJ.block(6*i, 6*i, 6, 6);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > esolver(mat_H);
        Eigen::Matrix<double, 1, 6> mat_E = esolver.eigenvalues().real(); // 6*1
        Eigen::Matrix<double, 6, 6> mat_V_f = esolver.eigenvectors().real(); // 6*6, column is the corresponding eigenvector
        Eigen::Matrix<double, 6, 6> mat_V_p = mat_V_f;
        for (auto j = 0; j < mat_E.cols(); j++)
        {
            if (mat_E(0, j) < eig_thre_[i])
            {
                mat_V_p.col(j) = Eigen::Matrix<double, 6, 1>::Zero();
                local_param_ids[i]->is_degenerate_ = true;
            } else
            {
                break;
            }
        }
        if (ESTIMATE_EXTRINSIC != 0)
            std::cout << i << " D factor: " << mat_E(0, 0) << ", D vector: " << mat_V_f.col(0).transpose() << std::endl;
        Eigen::Matrix<double, 6, 6> mat_P = (mat_V_f.transpose()).inverse() * mat_V_p.transpose(); // 6*6

        if (i > OPT_WINDOW_SIZE)
        {
            cur_eig_calib_[i - OPT_WINDOW_SIZE - 1] = mat_E(0, 0);
            if (mat_E(0, 0) >= EIG_THRE_CALIB)
            {
                eig_thre_[i] = EIG_THRE_CALIB;
                d_factor_calib_[i - OPT_WINDOW_SIZE - 1] = mat_E(0, 0);
            }
            else if (mat_E(0, 0) > eig_thre_[i])
                eig_thre_[i] = mat_E(0, 0);
            else
                mat_P.setZero(); // not update extrinsics
        }

        if (local_param_ids[i]->is_degenerate_)
        {
            local_param_ids[i]->V_update_ = mat_P;
            // std::cout << "param " << i << " is degenerate !" << std::endl;
            // std::cout << mat_P << std::endl;
        }
    }
    if (ESTIMATE_EXTRINSIC != 0)
    {
        printf("eigen threshold :");
        for (auto i = 0; i < eig_thre_.size(); i++) printf("%f ", eig_thre_[i]);
        printf("\n");
    }
    // printf("evaluate degeneracy %fms\n", t_eval_degenracy.toc());
}

void Estimator::evalCalib()
{
    if (solver_flag_ == NON_LINEAR)
    {
        for (auto n = 0; n < NUM_OF_LASER; n++)
        {
            if (d_factor_calib_[n] != 0) // with high constraints
            {
                double weight = pow(d_factor_calib_[n] / EIG_THRE_CALIB, 1.0);
                Pose pose_ext = Pose(qbl_[n], tbl_[n]);
                pose_calib_[n].push_back(make_pair(weight, pose_ext));
            }
        }

        // check if all lidars are coveraged
        bool is_converage = true;
        for (auto n = 0; n < NUM_OF_LASER; n++)
        {
            if (n == IDX_REF) continue;
            ROS_WARN("laser_%d, eligible calib size: %d", n, pose_calib_[n].size());
            if (pose_calib_[n].size() >= N_CALIB) calib_converge_[n] = true;
                                             else is_converage = false;
        }
        if (is_converage)
        {
            ROS_WARN("Finish nonlinear calibration !");
            ESTIMATE_EXTRINSIC = 0;
            for (auto n = 0; n < NUM_OF_LASER; n++)
            {
                if (n != IDX_REF)
                {
                    Pose pose_mean;
                    Eigen::Matrix<double, 6, 6> pose_cov;
                    computeMeanPose(pose_calib_[n], pose_mean, pose_cov); // compute the mean calibration parameters
                    qbl_[n] = pose_mean.q_;
                    tbl_[n] = pose_mean.t_;
                    // covbl_[n] = pose_cov;
                    covbl_[n] = pose_cov.diagonal().asDiagonal();
                    // std::cout << "laser_" << n << ": " << pose_mean_calib << std::endl;
                }
            }
            // ini_fixed_local_map_ = false; // reconstruct new optimized map
            if (last_marginalization_info_ != nullptr) delete last_marginalization_info_;
            last_marginalization_info_ = nullptr; // meaning that the prior errors in online calibration are discarded
            last_marginalization_parameter_blocks_.clear();
        }
    }
}

void Estimator::visualizePCL()
{
    // if (plane_normal_vis_.init_)
    // {
    //     PointCloud::Ptr point_world_xyz(new PointCloud);
    //     pcl::copyPointCloud(surf_points_local_map_filtered_[1], *point_world_xyz);
    //     plane_normal_vis_.UpdateCloud(point_world_xyz, "cloud_all");
    // }
    // int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    // std::vector<Eigen::Vector4d> plane_coeffs;
    // PointCloud::Ptr tmp_cloud_sel(new PointCloud); // surf_points_stack_[n][i]
    // NormalCloud::Ptr tmp_normals_sel(new NormalCloud); // surf_points_local_map_filtered_[n]
    // printf("feature size: %u\n", surf_map_features_[1][WINDOW_SIZE].size());
    // for (auto &f : surf_map_features_[1][WINDOW_SIZE])
    // {
    //     PointI p_ori;
    //     p_ori.x = f.point_.x();
    //     p_ori.y = f.point_.y();
    //     p_ori.z = f.point_.z();
    //     PointI p_sel;
    //     pointAssociateToMap(p_ori, p_sel, pose_local_[1][WINDOW_SIZE]);
    //     tmp_cloud_sel->push_back(Point{p_sel.x, p_sel.y, p_sel.z}); // target cloud
    //     tmp_normals_sel->push_back(Normal{float(f.coeffs_.x()), float(f.coeffs_.y()), float(f.coeffs_.z())}); // reference cloud normal
    //     // Eigen::Vector4d coeffs_normalized = f.coeffs;
    //     // double s_normal = coeffs_normalized.head<3>().norm();
    //     // coeffs_normalized = coeffs_normalized / s_normal;
    //     // plane_coeffs.push_back(coeffs_normalized);
    //     // DLOG(INFO) << p_sel.x * f.coeffs.x() + p_sel.y * f.coeffs.y() + p_sel.z * f.coeffs.z() + f.coeffs.w();
    // }
    // if (plane_normal_vis_.init_)
    // {
    //     plane_normal_vis_.UpdateCloudAndNormals(tmp_cloud_sel, tmp_normals_sel, PCL_VIEWER_NORMAL_RATIO, "cloud1", "normal1");
    // }
    // std::cout << "pose pivot to j: " << pose_local_[1][WINDOW_SIZE] << std::endl;
}

void Estimator::printParameter()
{
    printf("print optimized window (p -> j) [qx qy qz qw x y z]\n");
    for (auto i = 0; i < OPT_WINDOW_SIZE + 1; i++)
    {
        std::cout << "Pose " << WINDOW_SIZE - OPT_WINDOW_SIZE + i << ": " <<
            para_pose_[i][3] << " " <<
            para_pose_[i][4] << " " <<
            para_pose_[i][5] << " " <<
            para_pose_[i][6] << " " <<
            para_pose_[i][0] << " " <<
            para_pose_[i][1] << " " <<
            para_pose_[i][2] << std::endl;
    }
    for (auto i = 0; i < NUM_OF_LASER; i++)
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
    // for (auto i = 0; i < NUM_OF_LASER; i++)
    // {
    //     std::cout << "dt: " <<
    //         para_td_[i] << std::endl;
    // }
}

void Estimator::printSlideWindow()
{
    printf("print slide window (0 -> j) ************************\n");
    for (auto i = 0; i < cir_buf_cnt_ + 1; i++)
    {
        Pose pose(Qs_[i], Ts_[i]);
        std::cout << i << ": " << pose << std::endl;
    }
}

//
