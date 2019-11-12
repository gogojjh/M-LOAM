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

    pose_local_.clear();

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
    pose_local_.resize(NUM_OF_LASER);
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        surf_points_stack_[i].resize(WINDOW_SIZE + 1);
        surf_points_stack_size_[i].resize(WINDOW_SIZE + 1);
        // corner_points_stack_[i].resize(WINDOW_SIZE + 1);
        // corner_points_stack_size_[i].resize(WINDOW_SIZE + 1);
        pose_local_[i].resize(WINDOW_SIZE + 1);
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
        // printf("[LASER %u]: \n", i);
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
    const PointCloud &laser_cloud_in)
{
    input_cloud_cnt_++;
    m_buf_.lock();
    std::vector<cloudFeature> feature_frame;
    TicToc feature_ext_time;
    // printf("LASER 0: \n");
    feature_frame.push_back(f_extract_.extractCloud(t, laser_cloud_in));
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
        // -----------------
        // tracker and initialization
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
                printf("calibrating extrinsic param\n");
                printf("sufficient movement is needed \n");
                for (int i = 0; i < NUM_OF_LASER; i++)
                {
                    Pose calib_result;
                    // if (IDX_REF == i) continue;
                    if ((initial_extrinsics_.cov_rot_state_[i]) || (initial_extrinsics_.calibExRotation(IDX_REF, i, calib_result)))
                    {
                        printf("sufficient translation movement is needed\n");
                        initial_extrinsics_.setCovRotation(i);
                        if ((initial_extrinsics_.cov_pos_state_[i]) || (initial_extrinsics_.calibExTranslation(IDX_REF, i, calib_result)))
                        {
                            initial_extrinsics_.setCovTranslation(i);
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
                }
                if ((initial_extrinsics_.full_cov_rot_state_) && (initial_extrinsics_.full_cov_pos_state_))
                {
                    ROS_WARN("all initial extrinsic rotation calib success");
                    ESTIMATE_EXTRINSIC = 1;
                    initial_extrinsics_.saveStatistics();
                }
                printf("whole initialize extrinsics %fms\n", t_calib_ext.toc());
            }
        }
        // tracker
        else if (ESTIMATE_EXTRINSIC != 2)
        {
            cloudFeature &cur_cloud_feature = cur_feature_.second[IDX_REF];
            cloudFeature &prev_cloud_feature = prev_feature_.second[IDX_REF];
            pose_rlt_[IDX_REF] = lidar_tracker_.trackCloud(prev_cloud_feature, cur_cloud_feature, pose_rlt_[IDX_REF]);
            pose_laser_cur_[IDX_REF] = Pose(Qs_[cir_buf_cnt_-1], Ts_[cir_buf_cnt_-1]) * pose_rlt_[IDX_REF];
            std::cout << "relative transform: " << pose_rlt_[IDX_REF] << std::endl;
            std::cout << "current transform: " << pose_laser_cur_[IDX_REF] << std::endl;

            // Eigen::Vector3d ea = pose_rlt_[IDX_REF].T_.topLeftCorner<3, 3>().eulerAngles(2, 1, 0);
            // printf("relative euler (deg): %f, %f, %f\n", toDeg(ea(0)), toDeg(ea(1)), toDeg(ea(2)));

            printf("mloam_tracker %fms\n", t_mloam_tracker.toc());
        }
    }

    // get newest pose
    Qs_[cir_buf_cnt_] = pose_laser_cur_[IDX_REF].q_;
    Ts_[cir_buf_cnt_] = pose_laser_cur_[IDX_REF].t_;

    // get newest point cloud
    Header_[cir_buf_cnt_].stamp = ros::Time(cur_feature_.first);
    PointICloud cloud_downsampled_;
    for (int n = 0; n < NUM_OF_LASER; n++)
    {
        // PointICloud &corner_points = cur_feature_.second[n]["corner_points_less_sharp"];
        // f_extract_.down_size_filter_corner_.setInputCloud(boost::make_shared<PointICloud>(corner_points));
        // f_extract_.down_size_filter_corner_.filter(corner_points_stack_downsampled_);
        // corner_points_stack_[n].push(corner_points_stack_downsampled_);
        // corner_points_stack_size_[n].push(corner_points_stack_downsampled_.size());
        PointICloud &surf_points = cur_feature_.second[n]["surf_points_less_flat"];
        if (n == IDX_REF)
        {
            f_extract_.down_size_filter_surf_.setInputCloud(boost::make_shared<PointICloud>(surf_points));
            f_extract_.down_size_filter_surf_.filter(cloud_downsampled_);
            surf_points_stack_[n][cir_buf_cnt_] = cloud_downsampled_;
            surf_points_stack_size_[n][cir_buf_cnt_] = cloud_downsampled_.size();
        } else
        {
            f_extract_.down_size_filter_corner_.setInputCloud(boost::make_shared<PointICloud>(surf_points));
            f_extract_.down_size_filter_corner_.filter(cloud_downsampled_);
            surf_points_stack_[n][cir_buf_cnt_] = cloud_downsampled_;
            surf_points_stack_size_[n][cir_buf_cnt_] = cloud_downsampled_.size();
        }
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

            optimizeMap();
            slideWindow();

            // TODO
            // if checkNonLinearCalib()
            //     ESTIMATE_EXTRINSIC = 2;
            // ini_fixed_local_map_ = false;
            // last_marginalization_info_ = nullptr; // meaning that the prior errors in online calibration are discarded

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

void Estimator::optimizeMap()
{
    TicToc t_prep_solver, t_solver;
    size_t pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;

    // -----------------
    ceres::Problem problem;
    ceres::Solver::Summary summary;
    // ceres: set lossfunction and problem
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(0.5);
    // loss_function = new ceres::CauchyLoss(1.0);
    // ceres: set options and solve the non-linear equation
    ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 2;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    options.max_solver_time_in_seconds = SOLVER_TIME;

    // ****************************************************
    // ceres: add parameter block
    vector2Double();
    if (!OPTIMAL_ODOMETRY) printParameter();

    std::vector<double *> para_ids;
    for (size_t i = 0; i < OPT_WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_pose_[i], SIZE_POSE, local_parameterization);
        para_ids.push_back(para_pose_[i]);
    }
    problem.SetParameterBlockConstant(para_pose_[0]);

    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_ex_pose_[i], SIZE_POSE, local_parameterization);
        para_ids.push_back(para_ex_pose_[i]);
        if (ESTIMATE_EXTRINSIC != 0)
        {
            // ROS_INFO("extrinsic param are float");
        }
    }
    problem.SetParameterBlockConstant(para_ex_pose_[IDX_REF]);

    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        problem.AddParameterBlock(&para_td_[i], 1);
        para_ids.push_back(&para_td_[i]);
        if (ESTIMATE_TD != 0)
        {
            problem.SetParameterBlockConstant(&para_td_[i]);
        }
    }
    problem.SetParameterBlockConstant(&para_td_[IDX_REF]);

    // ****************************************************
    // ceres: add marginalization error of previous parameter blocks
    std::vector<ceres::internal::ResidualBlock *> res_ids_marg;
    if ((MARGINALIZATION_FACTOR) && (last_marginalization_info_))
    {
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
        ceres::internal::ResidualBlock *res_id_marg = problem.AddResidualBlock(marginalization_factor, NULL,
            last_marginalization_parameter_blocks_);
        res_ids_marg.push_back(res_id_marg);
    }

    // ****************************************************
    // ceres: add residual block within the sliding window
    std::vector<ceres::internal::ResidualBlock *> res_ids_proj;

    if (PRIOR_FACTOR)
    {
        for (int n = 0; n < NUM_OF_LASER; n++)
        {
            PriorFactor *f = new PriorFactor(tbl_[n], qbl_[n], PRIOR_FACTOR_POS, PRIOR_FACTOR_ROT);
            ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, NULL, para_ex_pose_[n]);
            res_ids_proj.push_back(res_id);
        }
    }

    // TODO: focus on online calibration
    if (ESTIMATE_EXTRINSIC == 1)
    {
        buildCalibMap();
        if (POINT_PLANE_FACTOR)
        {
            for (int n = 0; n < NUM_OF_LASER; n++)
            {
                for (size_t i = pivot_idx; i < WINDOW_SIZE + 1; i++)
                {
                    std::vector<PointPlaneFeature> &features_frame = surf_map_features_[n][i];
                    // printf("Laser_%d, Win_%d, features: %d\n", n, i, features_frame.size());
                    for (auto &feature: features_frame)
                    {
                        const double &s = feature.score_;
                        const Eigen::Vector3d &p_data = feature.point_;
                        const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                        if (n == IDX_REF)
                        {
                            // pivot pose optimization
                            if (i == pivot_idx) continue;
                            LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, s);
                            ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function,
                                para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[n]);
                            res_ids_proj.push_back(res_id);
                            if (CHECK_JACOBIAN)
                            {
                                double **tmp_param = new double *[3];
                                tmp_param[0] = para_pose_[0];
                                tmp_param[1] = para_pose_[i - pivot_idx];
                                tmp_param[2] = para_ex_pose_[n];
                                f->check(tmp_param);
                                CHECK_JACOBIAN = 0;
                            }
                        } else
                        {
                            // optimize extrinsics using local map
                            // continue;
                            if (i != pivot_idx) continue;
                            LidarPivotTargetPlaneNormFactor *f = new LidarPivotTargetPlaneNormFactor(p_data, coeff_ref, s, 1.0);
                            ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function,
                                para_ex_pose_[n]);
                            res_ids_proj.push_back(res_id);
                            if (CHECK_JACOBIAN)
                            {
                                double **tmp_param = new double *[3];
                                tmp_param[0] = para_pose_[0];
                                tmp_param[1] = para_pose_[i - pivot_idx];
                                tmp_param[2] = para_ex_pose_[n];
                                f->check(tmp_param);
                                CHECK_JACOBIAN = 0;
                            }
                        }
                    }
                }
            }

            // TODO: cumulative feature frame
            // if (cum_features_frame_ == 10)
            // {
            //     for (int n = 0; n < NUM_OF_LASER; n++)
            //     {
            //         if (n == IDX_REF) continue;
            //         for (auto &features: features_frame)
            //         {
            //             const double &s = feature.score_;
            //             const Eigen::Vector3d &p_data = feature.point_;
            //             const Eigen::Vector4d &coeff_ref = feature.coeffs_;
            //             LidarPivotTargetPlaneNormFactor *f = new LidarPivotTargetPlaneNormFactor(p_data, coeff_ref, s, 1.0);
            //             ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function, para_ex_pose_[n]);
            //             res_ids_proj.push_back(res_id);
            //         }
            //     }
            // }
        }
    }
    // TODO: focus on online odometry estimation
    else if (ESTIMATE_EXTRINSIC == 0)
    {
        for (int n = 0; n < NUM_OF_LASER; n++)
        {
            problem.SetParameterBlockConstant(para_ex_pose_[n]);
            // problem.SetParameterBlockConstant(&para_td_[n]);
        }
        buildLocalMap();
        if (POINT_PLANE_FACTOR)
        {
            for (int n = 0; n < NUM_OF_LASER; n++)
            {
                for (size_t i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
                {
                    std::vector<PointPlaneFeature> &features_frame = surf_map_features_[n][i];
                    // printf("Laser_%d, Win_%d, features: %d\n", n, i, features_frame.size());
                    for (auto &feature: features_frame)
                    {
                        const double &s = feature.score_;
                        const Eigen::Vector3d &p_data = feature.point_;
                        const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                        LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, s);
                        ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(f, loss_function,
                            para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[n]);
                        res_ids_proj.push_back(res_id);
                    }
                }
            }
        }
    }

    // *******************************
    printf("*************** Before optimization\n");
    printf("*************** Calib error\n");
    if (EVALUATE_RESIDUAL) evalResidual(problem, para_ids, res_ids_proj, last_marginalization_info_, res_ids_marg);

    printf("prepare ceres %fms\n", t_prep_solver.toc()); // cost time
    if (!OPTIMAL_ODOMETRY) return;

    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    // std::cout << summary.FullReport() << std::endl;
    // printf("Iterations : %d\n", static_cast<int>(summary.iterations.size()));
    printf("Solver costs: %fms\n", t_solver.toc());

    printf("*************** After optimization\n");
    printf("*************** Calib error\n");
    if (EVALUATE_RESIDUAL) evalResidual(problem, para_ids, res_ids_proj, last_marginalization_info_, res_ids_marg);

    double2Vector();
    printParameter();

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

        if (PRIOR_FACTOR)
        {
            for (int n = 0; n < NUM_OF_LASER; n++)
            {
                PriorFactor *f = new PriorFactor(tbl_[n], qbl_[n], PRIOR_FACTOR_POS, PRIOR_FACTOR_ROT);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, NULL,
                    std::vector<double *>{para_ex_pose_[n]}, std::vector<int>{0});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // TODO: add marginalization block
        if (ESTIMATE_EXTRINSIC == 1)
        {
            if (POINT_PLANE_FACTOR)
            {
                for (int n = 0; n < NUM_OF_LASER; n++)
                {
                    for (size_t i = pivot_idx; i < WINDOW_SIZE + 1; i++)
                    {
                        std::vector<PointPlaneFeature> &features_frame = surf_map_features_[n][i];
                        for (auto &feature: features_frame)
                        {
                            const double &s = feature.score_;
                            const Eigen::Vector3d &p_data = feature.point_;
                            const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                            if (n == IDX_REF)
                            {
                                if (i == pivot_idx) continue;
                                LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, s);
                                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                    std::vector<double *>{para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[n]}, std::vector<int>{0});
                                marginalization_info->addResidualBlockInfo(residual_block_info);
                            } else
                            {
                                if (i != pivot_idx) continue;
                                LidarPivotTargetPlaneNormFactor *f = new LidarPivotTargetPlaneNormFactor(p_data, coeff_ref, s, 1.0);
                                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                    std::vector<double *>{para_ex_pose_[n]}, std::vector<int>{0});
                                marginalization_info->addResidualBlockInfo(residual_block_info);
                            }
                        }
                    }
                }
            }
        }
        else if (ESTIMATE_EXTRINSIC == 0)
        {
            if (POINT_PLANE_FACTOR)
            {
                for (int n = 0; n < NUM_OF_LASER; n++)
                {
                    for (size_t i = pivot_idx + 1; i < WINDOW_SIZE + 1; i++)
                    {
                        std::vector<PointPlaneFeature> &features_frame = surf_map_features_[n][i];
                        for (auto &feature: features_frame)
                        {
                            const double &s = feature.score_;
                            const Eigen::Vector3d &p_data = feature.point_;
                            const Eigen::Vector4d &coeff_ref = feature.coeffs_;
                            LidarPivotPlaneNormFactor *f = new LidarPivotPlaneNormFactor(p_data, coeff_ref, s);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                vector<double *>{para_pose_[0], para_pose_[i - pivot_idx], para_ex_pose_[n]}, std::vector<int>{0});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        //! calculate the residuals and jacobian of all ResidualBlockInfo over the marginalized parameter blocks,
        //! for next iteration, the linearization point is assured and fixed
        //! adjust the memory of H and b to implement the Schur complement
        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        printf("pre marginalization: %fms\n", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        printf("marginalization: %fms\n", t_margin.toc());

        // TODO:
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
        printf("whole marginalization costs: %fms\n", t_whole_marginalization.toc());
    }
}

void Estimator::buildCalibMap()
{
    TicToc t_build_map;

    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    Pose pose_pivot(Qs_[pivot_idx], Ts_[pivot_idx]);
    // -----------------
    // build static local map using fixed poses
    PointICloud surf_points_trans;
    if (!ini_fixed_local_map_)
    {
        PointICloud surf_points_tmp;
        for (size_t i = 0; i <= pivot_idx; i++)
        {
            Pose pose_i(Qs_[i], Ts_[i]);
            Eigen::Affine3d transform_pivot_i;
            transform_pivot_i.matrix() = pose_pivot.T_ .inverse() * pose_i.T_;
            pcl::transformPointCloud(surf_points_stack_[IDX_REF][i], surf_points_trans, transform_pivot_i.cast<float>());
            for (auto &p: surf_points_trans.points) p.intensity = i;
            surf_points_tmp += surf_points_trans;
            // pcl::transformPointCloud(corner_points_stack_[n][idx], corner_points_transformed, transform_pivot_i);
            // corner_points_tmp[n] += corner_points_transformed;
        }
        surf_points_stack_[IDX_REF][pivot_idx] = surf_points_tmp;
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

    for (int n = 0; n < NUM_OF_LASER; n++)
    {
        Pose pose_ext = Pose(qbl_[n], tbl_[n]);
        for (int i = 0; i < WINDOW_SIZE + 1; i++)
        {
            Pose pose_i(Qs_[i], Ts_[i]);
            Eigen::Affine3d transform_pivot_i;
            transform_pivot_i.matrix() = pose_pivot.T_.inverse() * pose_i.T_ * pose_ext.T_;
            pose_local_[n][i] = Pose(transform_pivot_i.matrix());
            if ((i < pivot_idx) || (i == WINDOW_SIZE)) continue;
            if (n == IDX_REF)
            {
                pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_trans, transform_pivot_i.cast<float>());
                for (auto &p: surf_points_trans.points) p.intensity = i;
                surf_points_local_map_[n] += surf_points_trans;
            }
        }
        pcl::VoxelGrid<PointI> down_size_filter;
        if (n == IDX_REF)
            down_size_filter.setLeafSize(0.4, 0.4, 0.4);
        else
            down_size_filter.setLeafSize(0.2, 0.2, 0.2);
        down_size_filter.setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_[IDX_REF]));
        down_size_filter.filter(surf_points_local_map_filtered_[n]);
        // f_extract_.down_size_filter_corner_.setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_));
        // f_extract_.down_size_filter_corner_.filter(corner_points_local_map_filtered_);
        // printf("Laser_%d, filtered local map %d -> %d\n", n, surf_points_local_map_[n].size(), surf_points_local_map_filtered_[n].size());
    }

    // -----------------
    // calculate features and correspondences from p+1 to j
    TicToc t_map_extract;
    surf_map_features_.clear();
    surf_map_features_.resize(NUM_OF_LASER);
    for (int n = 0; n < NUM_OF_LASER; n++)
    {
        surf_map_features_[n].resize(WINDOW_SIZE + 1);
        pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_points_local_map(new pcl::KdTreeFLANN<PointI>());
        kdtree_surf_points_local_map->setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_filtered_[n]));
        // pcl::KdTreeFLANN<PointI>::Ptr kdtree_corner_points_local_map(new pcl::KdTreeFLANN<PointI>());
        // kdtree_corner_points_local_map->setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_filtered_[n]));
        int n_neigh = (n==IDX_REF ? 5:10);
        for (size_t i = pivot_idx; i < WINDOW_SIZE + 1; i++)
        {
            f_extract_.extractSurfFromMap(kdtree_surf_points_local_map, surf_points_local_map_filtered_[n],
                surf_points_stack_[n][i], pose_local_[n][i], surf_map_features_[n][i], n_neigh);
        }
    }
    // printf("extract map: %fms\n", t_map_extract.toc());

    printf("build map: %fms\n", t_build_map.toc());

    if (PCL_VIEWER) visualizePCL();
}

void Estimator::buildLocalMap()
{
    TicToc t_build_map;

    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    Pose pose_pivot(Qs_[pivot_idx], Ts_[pivot_idx]);
    // -----------------
    // build static local map using fixed poses
    PointICloud surf_points_trans;
    if (!ini_fixed_local_map_)
    {
        PointICloud surf_points_tmp;
        for (int n = 0; n < NUM_OF_LASER; n++)
        {
            Pose pose_ext = Pose(qbl_[n], tbl_[n]);
            for (size_t i = 0; i <= pivot_idx; i++)
            {
                Pose pose_i(Qs_[i], Ts_[i]);
                Eigen::Affine3d transform_pivot_i;
                transform_pivot_i.matrix() = (pose_pivot.T_ * pose_ext.T_).inverse() * (pose_i.T_ * pose_ext.T_);
                pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_trans, transform_pivot_i.cast<float>());
                for (auto &p: surf_points_trans.points) p.intensity = i;
                surf_points_tmp += surf_points_trans;
                // pcl::transformPointCloud(corner_points_stack_[n][idx], corner_points_transformed, transform_pivot_i);
                // corner_points_tmp[n] += corner_points_transformed;
            }
            surf_points_stack_[n][pivot_idx] = surf_points_tmp;
            // corner_points_stack_[n][pivot_idx] = corner_points_tmp[n];
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

    for (int n = 0; n < NUM_OF_LASER; n++)
    {
        Pose pose_ext = Pose(qbl_[n], tbl_[n]);
        for (int i = 0; i < WINDOW_SIZE + 1; i++)
        {
            Pose pose_i(Qs_[i], Ts_[i]);
            Eigen::Affine3d transform_pivot_i;
            transform_pivot_i.matrix() = (pose_pivot.T_ * pose_ext.T_).inverse() * (pose_i.T_ * pose_ext.T_);
            pose_local_[n][i] = Pose(transform_pivot_i.matrix());
            if ((i < pivot_idx) || (i == WINDOW_SIZE)) continue;

            pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_trans, transform_pivot_i.cast<float>());
            for (auto &p: surf_points_trans.points) p.intensity = i;
            surf_points_local_map_[n] += surf_points_trans;
            // pcl::transformPointCloud(corner_points_stack_[n][idx], corner_points_transformed, transform_pivot_i);
            // for (auto &p: corner_points_transformed.points) p.intensity = i;
            // corner_points_local_map_[n] += corner_points_transformed;
        }
        f_extract_.down_size_filter_local_map_.setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_[n]));
        f_extract_.down_size_filter_local_map_.filter(surf_points_local_map_filtered_[n]);
        // f_extract_.down_size_filter_corner_.setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_));
        // f_extract_.down_size_filter_corner_.filter(corner_points_local_map_filtered_);
        // printf("Laser_%d, filtered local map %d -> %d\n", n, surf_points_local_map_[n].size(), surf_points_local_map_filtered_[n].size());
    }
    printf("build local map: %fms\n", t_build_map.toc());

    // -----------------
    // calculate features and correspondences from p+1 to j
    TicToc t_map_extract;
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
                surf_points_stack_[n][i], pose_local_[n][i], surf_map_features_[n][i]);
        }
    }
    printf("extract map: %fms\n", t_map_extract.toc());

    if (PCL_VIEWER) visualizePCL();
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

        int i = pivot_idx + 1;
        Pose pose_i(Qs_[i], Ts_[i]);
        for (int n = 0; n < NUM_OF_LASER; n++)
        {
            if ((ESTIMATE_EXTRINSIC == 1) && (n != IDX_REF)) continue; // only filter the reference local map

            surf_points_pivot_map_[n] = surf_points_stack_[n][pivot_idx];
            PointICloud surf_points_trans, surf_points_filtered;
            // PointICloud corner_points_transformed, corner_points_filtered;
            Pose pose_ext = Pose(qbl_[n], tbl_[n]);
            Eigen::Affine3d transform_i_pivot;
            transform_i_pivot.matrix() = (pose_i.T_ * pose_ext.T_).inverse() * (pose_pivot.T_ * pose_ext.T_);
            pcl::transformPointCloud(surf_points_stack_[n][pivot_idx], surf_points_trans, transform_i_pivot.cast<float>());
            pcl::PointIndices::Ptr inliers_surf(new pcl::PointIndices());
            for (size_t j = 0; j < surf_points_stack_size_[n][0]; j++) inliers_surf->indices.push_back(j);
            pcl::ExtractIndices<PointI> extract;
            extract.setInputCloud(boost::make_shared<PointICloud>(surf_points_trans));
            extract.setIndices(inliers_surf);
            extract.setNegative(true);
            extract.filter(surf_points_filtered);
            surf_points_filtered += surf_points_stack_[n][i];
            surf_points_stack_[n][i] = surf_points_filtered;
        }
    }

    Qs_.push(Qs_[cir_buf_cnt_]);
    Ts_.push(Ts_[cir_buf_cnt_]);
    Header_.push(Header_[cir_buf_cnt_]);
    for (int n = 0; n < NUM_OF_LASER; n++)
    {
        surf_points_stack_[n].push(surf_points_stack_[n][cir_buf_cnt_]);
        surf_points_stack_size_[n].push(surf_points_stack_size_[n][cir_buf_cnt_]);
    }
    printf("slide window: %fms\n", t_solid_window.toc());
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

void Estimator::evalResidual(ceres::Problem &problem,
    const std::vector<double *> &para_ids,
    const std::vector<ceres::internal::ResidualBlock *> &res_ids_proj,
	const MarginalizationInfo *last_marginalization_info_,
    const std::vector<ceres::internal::ResidualBlock *> &res_ids_marg)
{
	double cost = 0.0;
    ceres::Problem::EvaluateOptions e_option;
	if (POINT_PLANE_FACTOR)
	{
		e_option.parameter_blocks = para_ids;
		e_option.residual_blocks = res_ids_proj;
		problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
		printf("residual proj: %f, ", cost);
	}
	if (MARGINALIZATION_FACTOR)
	{
		if (last_marginalization_info_ && !res_ids_marg.empty())
		{
			e_option.parameter_blocks = para_ids;
			e_option.residual_blocks = res_ids_marg;
			problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
			printf("residual marg: %f \nn", cost);
		}
	}
}

void Estimator::visualizePCL()
{
    if (plane_normal_vis_.init_)
    {
        PointCloud::Ptr point_world_xyz(new PointCloud);
        pcl::copyPointCloud(surf_points_local_map_filtered_[1], *point_world_xyz);
        plane_normal_vis_.UpdateCloud(point_world_xyz, "cloud_all");
    }
    int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > plane_coeffs;
    PointCloud::Ptr tmp_cloud_sel(new PointCloud); // surf_points_stack_[n][i]
    NormalCloud::Ptr tmp_normals_sel(new NormalCloud); // surf_points_local_map_filtered_[n]
    printf("feature size: %d\n", surf_map_features_[1][pivot_idx + 1].size());
    for (auto &f : surf_map_features_[1][pivot_idx + 1])
    {
        PointI p_ori;
        p_ori.x = f.point_.x();
        p_ori.y = f.point_.y();
        p_ori.z = f.point_.z();
        PointI p_sel;
        f_extract_.pointAssociateToMap(p_ori, p_sel, pose_local_[1][pivot_idx + 1]);
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
    std::cout << "pose pivot to j: " << pose_local_[1][pivot_idx + 1] << std::endl;
}

void Estimator::printParameter()
{
    printf("print optimized window (p -> j) [qx qy qz qw x y z]\n");
    for (size_t i = 0; i < OPT_WINDOW_SIZE + 1; i++)
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
