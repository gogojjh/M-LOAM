void LidarOptimizer::Vector2Double()
{

}

void LidarOptimizer::AddPose()
{

}

void LidarOptimizer::OptimizeLocalMap()
{
    if (cir_buf_cnt_ < WINDOW_SIZE)
    {
        ROS_WARN("enter optimization before enough count: %d < %d", cir_buf_cnt_, WINDOW_SIZE);
        return;
    }
    TicToc t_opt_local_map;

    Vector2Double();

    // -----------------
    // ceres: set lossfunction and problem
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(0.5);
    loss_function = new ceres::CauchyLoss(1.0);
    ceres::Problem problem;

    // -----------------
    // ceres: add parameter block
    for (size_t i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_pose_[i], SIZE_POSE, local_parameterization);
    }
    for (size_t i = 0; i < NUM_OF_LASER; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_ex_pose_[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC) && (OPT_EXTRINSIC))
        {
            ROS_INFO("estimate extrinsic param");
        } else
        {
            ROS_INFO("fix extrinsic param");
            problem.setParameterBlockConstant(para_ex_pose_[i]);
        }
    }
    problem.AddParameterBlock(para_td_[0], 1);
    if (!ESTIMATE_TD)
    {
        problem.setParameterBlockConstant(para_td_[0]);
    }

    // -----------------
    // ceres: add marginalization factor
    if ((MARGINALIZATION_FACTOR) && (last_marginalization_info_))
    {
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
        res_id_marg = problem.AddParameterBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks_);
        res_ids_marg.push_back(res_id_marg);
    }

    // -----------------
    // ceres: add residual block within the sliding window


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
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;

    // TODO: calculate region residual before optimization
    // RegionResidual();

    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    //printf("solver costs: %f \n", t_solver.toc());

    // TODO: calculate region residual after optimization
    // RegionResidual();

    Double2Vector();

    // -----------------
    // ceres: perform marginalization
    if (MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        Vector2Double();
        if (last_marginalization_info_)
        {
            std::vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++)
            {
                if (last_marginalization_parameter_blocks_[i] == para_pose_[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                           last_marginalization_parameter_blocks_,
                                                           drop_set);
            marginalization_info->AddResidualBlockInfo(residual_block_info);
        }

        if (POINT_DISTANCE_FACTOR)
        {

        }

        TicToc t_pre_margin;
        marginalization_info->PreMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.Toc());
        ROS_DEBUG_STREAM("pre marginalization: " << t_pre_margin.Toc() << " ms");

        TicToc t_margin;
        marginalization_info->Marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.Toc());
        ROS_DEBUG_STREAM("marginalization: " << t_margin.Toc() << " ms");

        // TODO:
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i < estimator_config_.opt_window_size + 1; ++i)
        {
            addr_shift[reinterpret_cast<long>(para_pose_[i])] = para_pose_[i - 1];
            addr_shift[reinterpret_cast<long>(para_speed_bias_[i])] = para_speed_bias_[i - 1];
        }
        addr_shift[reinterpret_cast<long>(para_ex_pose_)] = para_ex_pose_;

        vector<double *> parameter_blocks = marginalization_info->GetParameterBlocks(addr_shift);

        if (last_marginalization_info_)
        {
            delete last_marginalization_info_;
        }
        last_marginalization_info_ = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;

        DLOG(INFO) << "whole marginalization costs: " << t_whole_marginalization.Toc();
        ROS_DEBUG_STREAM("whole marginalization costs: " << t_whole_marginalization.Toc() << " ms");
    }

    // -----------------
    // update optimized transform and sliding window


    ROS_DEBUG("opt local map costs: %fms", t_opt_local_map.toc());
}





//
