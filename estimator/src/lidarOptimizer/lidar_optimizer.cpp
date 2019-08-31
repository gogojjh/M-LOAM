//
// using namespace common;
//
// LidarOptimizer::LidarOptimizer()
// {
//     down_size_filter_corner_.setLeafSize(0.2, 0.2, 0.2);
//     down_size_filter_surf_.setLeafSize(0.4, 0.4, 0.4);
//     down_size_filter_map_.setLeafSize(0.6, 0.6, 0.6);
// }
//
// void LidarOptimizer::vector2Double()
// {
//     for (size_t i = 0; i < WINDOW_SIZE; i++)
//     {
//         para_pose_[i][0] = pose_[i].q_.x();
//         para_pose_[i][1] = pose_[i].q_.y();
//         para_pose_[i][2] = pose_[i].q_.z();
//         para_pose_[i][3] = pose_[i].q_.w();
//         para_pose_[i][4] = pose_[i].t_(0);
//         para_pose_[i][5] = pose_[i].t_(1);
//         para_pose_[i][6] = pose_[i].t_(2);
//     }
//     for (size_t i = 0; i < NUM_OF_LASER; i++)
//     {
//         para_ex_pose_[i][0] = calib_[i].q_.x();
//         para_ex_pose_[i][1] = calib_[i].q_.y();
//         para_ex_pose_[i][2] = calib_[i].q_.z();
//         para_ex_pose_[i][3] = calib_[i].q_.w();
//         para_ex_pose_[i][4] = calib_[i].t_(0);
//         para_ex_pose_[i][5] = calib_[i].t_(1);
//         para_ex_pose_[i][6] = calib_[i].t_(2);
//     }
// }
//
// void LidarOptimizer::double2Vector()
// {
//
// }
//
// void LidarOptimizer::buildLocalMap()
// {
//     TicToc t_build_local_map;
//
//     int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
//     Pose pose_pivot(qs_[pivot_idx], ts_[pivot_idx]);
//
//     PointICloud surf_points_transformed;
//
//     // -----------------
//     // build static local map using poses before pivot_pose
//     if (!ini_local_map_)
//     {
//         for (size_t n = 0; n < NUM_OF_LASER; n++)
//         {
//             PointICloud surf_points_tmp;
//             Pose pose_ext = Pose(estimator.qbl_[n], estimator.tbl_[n]);
//             for (size_t i = 0; i <= pivot_idx; i++)
//             {
//                 Pose pose_i(qs_[i], ts_[i]);
//                 Eigen::Affin3f transform_pivot_i((pose_pivot.T_ * pose_ext.T_).inverse() * (pose_i.T_ * pose_ext.T_)).cast<float>());
//                 pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_transformed, transform_pivot_i);
//                 surf_points_tmp += surf_points_transformed;
//                 // pcl::transformPointCloud(corner_points_stack_[n][idx], corner_points_transformed, transform_pivot_i);
//                 // corner_points_tmp[n] += corner_points_transformed;
//             }
//             surf_points_stack_[n][pivot_idx] = surf_points_tmp;
//             // corner_points_stack_[n][pivot_idx] = corner_points_tmp[n];
//         }
//         ini_local_map_ = true;
//     }
//
//     // -----------------
//     // build changing local map using all poses
//     for (size_t n = 0; n < NUM_OF_LASER; n++)
//     {
//         surf_points_local_map_[n].clear();
//         surf_points_local_map_filtered_[n].clear();
//         // corner_points_local_map_[n].clear();
//         // corner_points_local_map_filtered_[n].clear();
//     }
//     for (size_t n = 0; n < NUM_OF_LASER; n++)
//     {
//         Pose pose_ext = Pose(estimator.qbl_[n], estimator.tbl_[n]);
//         for (size_t i = pivot_idx; i < WINDOW_SIZE; i++)
//         {
//             Pose pose_i(qs_[i], ts_[i]);
//             Eigen::Affin3f transform_pivot_i((pose_pivot.T_ * pose_ext.T_).inverse() * (pose_i.T_ * pose_ext.T_)).cast<float>());
//             pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_transformed, transform_pivot_i);
//             for (auto &p: surf_points_transformed.points) p.intensity = 255.0 * i / WINDOW_SIZE;
//             surf_points_local_map_[n] += surf_points_transformed;
//             pcl::transformPointCloud(corner_points_stack_[n][idx], corner_points_transformed, transform_pivot_i);
//             // for (auto &p: corner_points_transformed.points) p.intensity = 255.0 * i / WINDOW_SIZE;
//             // corner_points_local_map_[n] += corner_points_transformed;
//         }
//         down_size_filter_surf_.setInputCloud(boost::make_share<PointICloud>(surf_points_local_map_[n]));
//         down_size_filter_surf_.filter(surf_points_local_map_filtered_[n]);
//         // down_size_filter_corner_.setInputCloud(boost::make_share<PointICloud>(corner_points_local_map_));
//         // down_size_filter_corner_.filter(corner_points_local_map_filtered_);
//     }
//     printf("build local map: %fms\n", t_build_local_map.toc());
//
//     // -----------------
//     // calculate features
//     for (size_t n = 0; n < NUM_OF_LASER; n++)
//     {
//         // pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_points_local_map(new pcl::KdTreeFLANN<PointI>());
//         // kdtree_surf_points_local_map.setInputCloud(boost::make_share<PointICloud>(surf_points_local_map_filtered_[n]);
//         // for (size_t i = pivot_idx; i < WINDOW_SIZE; i++)
//         // {
//         //     if (i != WINDOW_SIZE)
//         //     {
//         //         f_extract_.extractLocalMap(kdtree_surf_points_local_map, surf_points_local_map_filtered_[n],
//         //             surf_points_stack_[n][i], features);
//         //     } else
//         //     {
//         //         calculateDistance(surf_points_local_map_filtered_[n], surf_points_stack_[n][idx]);
//         //     }
//         //
//         // }
//     }
//
// }
//
// // push new state and measurements in the sliding window
// // move the localmap in the pivot frame to the pivot+1 frame, and remove the first point cloud
// void LidarOptimizer::slidingWindow()
// {
//     if (ini_local_map_)
//     {
//         int pivot_idx = WINDOW_SIZE - OPT_WINDOW_SIZE;
//         Pose pose_pivot(qs_[pivot_idx], ts_[pivot_idx]);
//
//         PointICloud surf_points_transformed, surf_points_filtered;
//         // PointICloud corner_points_transformed, corner_points_filtered;
//
//         int i = pivot_idx + 1;
//         Pose pose_i(qs_[i], ts_[i]);
//         for (size_t n = 0; n < NUM_OF_LASER; i++)
//         {
//             Pose pose_ext = Pose(estimator.qbl_[n], estimator.tbl_[n]);
//             Eigen::Affin3f transform_i_pivot(((pose_i.T_ * pose_ext.T_).inverse() * (pose_pivot.T_ * pose_ext.T_)).cast<float>());
//             pcl::transformPointCloud(surf_points_stack_[n][pivot_idx], surf_points_transformed, transform_i_pivot);
//
//             pcl::PointIndices::Ptr inliers_surf(new pcl::PointIndices());
//             for (size_t j = 0; j < size_surf_points_stack_[n]; j++) inliers_surf->indices.push_back(j);
//             pcl::ExtractIndices<PointI> extract;
//             extract.setInputCloud(boost::make_share<PointICloud>(surf_points_transformed));
//             extract.setIndices(inliers_surf);
//             extract.setNegative(true);
//             extract.filter(surf_points_filtered);
//             surf_points_filtered += surf_points_stack_[n][i];
//             surf_points_stack_[n][i] = surf_points_filtered;
//         }
//     }
//     qs_.push(qs_[cir_buf_cnt_]);
//     ts_.push(ts_[cir_buf_cnt_]);
// }
//
// void LidarOptimizer::OptimizeLocalMap()
// {
//     if (cir_buf_cnt_ < WINDOW_SIZE)
//     {
//         ROS_WARN("enter optimization before enough count: %d < %d", cir_buf_cnt_, WINDOW_SIZE);
//         return;
//     }
//     TicToc t_opt_local_map;
//
//     Vector2Double();
//
//     // -----------------
//     // ceres: set lossfunction and problem
//     ceres::LossFunction *loss_function;
//     // loss_function = new ceres::HuberLoss(0.5);
//     loss_function = new ceres::CauchyLoss(1.0);
//     ceres::Problem problem;
//
//     // -----------------
//     // ceres: add parameter block
//     for (size_t i = 0; i < WINDOW_SIZE + 1; i++)
//     {
//         ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
//         problem.AddParameterBlock(para_pose_[i], SIZE_POSE, local_parameterization);
//     }
//     for (size_t i = 0; i < NUM_OF_LASER; i++)
//     {
//         ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
//         problem.AddParameterBlock(para_ex_pose_[i], SIZE_POSE, local_parameterization);
//         if ((ESTIMATE_EXTRINSIC) && (OPT_EXTRINSIC))
//         {
//             ROS_INFO("estimate extrinsic param");
//         } else
//         {
//             ROS_INFO("fix extrinsic param");
//             problem.setParameterBlockConstant(para_ex_pose_[i]);
//         }
//     }
//     int and
//
//     problem.AddParameterBlock(para_td_[0], 1);
//     if (!ESTIMATE_TD)
//     {
//         problem.setParameterBlockConstant(para_td_[0]);
//     }
//
//     // -----------------
//     // ceres: add marginalization factor
//     if ((MARGINALIZATION_FACTOR) && (last_marginalization_info_))
//     {
//         MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
//         res_id_marg = problem.AddParameterBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks_);
//         res_ids_marg.push_back(res_id_marg);
//     }
//
//     // -----------------
//     // ceres: add residual block within the sliding window
//
//
//     // -----------------
//     // ceres: set options and solve the non-linear equation
//     ceres::Solver::Options options;
//     options.linear_solver_type = ceres::DENSE_SCHUR;
//     //options.num_threads = 2;
//     options.trust_region_strategy_type = ceres::DOGLEG;
//     // options.trust_region_strategy_type = ceres::LEVENBEGR_MARQUARDT;
//     options.max_num_iterations = NUM_ITERATIONS;
//     //options.use_explicit_schur_complement = true;
//     //options.minimizer_progress_to_stdout = true;
//     //options.use_nonmonotonic_steps = true;
//     if (marginalization_flag == MARGIN_OLD)
//     options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
//     else
//     options.max_solver_time_in_seconds = SOLVER_TIME;
//
//     // TODO: calculate region residual before optimization
//     // RegionResidual();
//
//     TicToc t_solver;
//     ceres::Solver::Summary summary;
//     ceres::Solve(options, &problem, &summary);
//     //cout << summary.BriefReport() << endl;
//     ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
//     //printf("solver costs: %f \n", t_solver.toc());
//
//     // TODO: calculate region residual after optimization
//     // RegionResidual();
//
//     Double2Vector();
//
//     // -----------------
//     // ceres: perform marginalization
//     if (MARGIN_OLD)
//     {
//         MarginalizationInfo *marginalization_info = new MarginalizationInfo();
//         Vector2Double();
//         if (last_marginalization_info_)
//         {
//             std::vector<int> drop_set;
//             for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks_.size()); i++)
//             {
//                 if (last_marginalization_parameter_blocks_[i] == para_pose_[0])
//                 drop_set.push_back(i);
//             }
//             // construct new marginlization_factor
//             MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info_);
//             ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
//                 last_marginalization_parameter_blocks_,
//                 drop_set);
//                 marginalization_info->AddResidualBlockInfo(residual_block_info);
//             }
//
//             if (POINT_DISTANCE_FACTOR)
//             {
//
//             }
//
//             TicToc t_pre_margin;
//             marginalization_info->PreMarginalize();
//             ROS_DEBUG("pre marginalization %f ms", t_pre_margin.Toc());
//             ROS_DEBUG_STREAM("pre marginalization: " << t_pre_margin.Toc() << " ms");
//
//             TicToc t_margin;
//             marginalization_info->Marginalize();
//             ROS_DEBUG("marginalization %f ms", t_margin.Toc());
//             ROS_DEBUG_STREAM("marginalization: " << t_margin.Toc() << " ms");
//
//             // TODO:
//             std::unordered_map<long, double *> addr_shift;
//             for (int i = 1; i < estimator_config_.opt_window_size + 1; ++i)
//             {
//                 addr_shift[reinterpret_cast<long>(para_pose_[i])] = para_pose_[i - 1];
//                 addr_shift[reinterpret_cast<long>(para_speed_bias_[i])] = para_speed_bias_[i - 1];
//             }
//             addr_shift[reinterpret_cast<long>(para_ex_pose_)] = para_ex_pose_;
//
//             vector<double *> parameter_blocks = marginalization_info->GetParameterBlocks(addr_shift);
//
//             if (last_marginalization_info_)
//             {
//                 delete last_marginalization_info_;
//             }
//             last_marginalization_info_ = marginalization_info;
//             last_marginalization_parameter_blocks = parameter_blocks;
//
//             DLOG(INFO) << "whole marginalization costs: " << t_whole_marginalization.Toc();
//             ROS_DEBUG_STREAM("whole marginalization costs: " << t_whole_marginalization.Toc() << " ms");
//         }
//
//         // -----------------
//         // update optimized transform and sliding window
//
//
//         ROS_DEBUG("opt local map costs: %fms", t_opt_local_map.toc());
//     }
//
//
//
//
//
//     //
