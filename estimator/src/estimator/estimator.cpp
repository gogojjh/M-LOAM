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
    corner_points_local_map_.clear();
    corner_points_local_map_filtered_.clear();

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
    corner_points_local_map_.resize(NUM_OF_LASER);
    corner_points_local_map_filtered_.resize(NUM_OF_LASER);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !init_thread_flag_)
    {
        init_thread_flag_ = true;
        process_thread_ = std::thread(&Estimator::processMeasurements, this);
    }

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
        printf("[LASER %d]: \n", i);
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

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void Estimator::process()
{
    // -----------------
    if (ESTIMATE_EXTRINSIC == 2)
    {
        // feature tracker: estimate the relative transformations of each lidar
        if (!b_system_inited_)
        {
            b_system_inited_ = true;
            printf("System initialization finished \n");
            for (size_t i = 0; i < NUM_OF_LASER; i++)
            {
                pose_rlt_[i] = Pose();
                // pose_prev_cur_[i].push_back(Pose());
                pose_laser_cur_[i] = Pose();
            }
        } else
        {
            TicToc t_mloam_tracker;
            for (size_t i = 0; i < NUM_OF_LASER; i++)
            {
                printf("[LASER %d]:\n", i);
                cloudFeature &cur_cloud_feature = cur_feature_.second[i];
                cloudFeature &prev_cloud_feature = prev_feature_.second[i];
                // Pose pose_rlt = lidar_tracker_.trackCloud(prev_cloud_feature, cur_cloud_feature, *(pose_prev_cur_[i].end()-1));
                // pose_prev_cur_[i].push_back(pose_rlt);
                pose_rlt_[i] = lidar_tracker_.trackCloud(prev_cloud_feature, cur_cloud_feature, pose_rlt_[i]);
                pose_laser_cur_[i] = pose_laser_cur_[i] * pose_rlt_[i];
                std::cout << "relative transform: " << pose_rlt_[i] << std::endl;
                std::cout << "current transform: " << pose_laser_cur_[i] << std::endl;
            }
            printf("mloam_tracker %fms\n", t_mloam_tracker.toc());
        }
        // initialize extrinsics
        for (size_t i = 0; i < NUM_OF_LASER; i++) initial_extrinsics_.addPose(pose_rlt_[i], i);
        if (cir_buf_cnt_ == WINDOW_SIZE)
        {
            TicToc t_calib_ext;
            printf("calibrating extrinsic param, rotation movement is needed \n");
            for (size_t i = 0; i < NUM_OF_LASER; i++)
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

    // get newest point cloud
    Header_[cir_buf_cnt_].stamp = ros::Time(cur_feature_.first);
    PointICloud cloud_downsampled_;
    for (size_t i = 0; i < NUM_OF_LASER; i++)
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

    // -----------------
    switch (solver_flag_)
    {
        // INITIAL: multi-LiDAR individual tracking
        case INITIAL:
        {
            // Qs[cir_buf_cnt_] = Qs.last()
            Qs_[cir_buf_cnt_] = pose_laser_cur_[IDX_REF].q_;
            Ts_[cir_buf_cnt_] = pose_laser_cur_[IDX_REF].t_;
            slideWindow();
            if (cir_buf_cnt_ < WINDOW_SIZE)
            {
                cir_buf_cnt_++;
            } else
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
            TicToc t_solve;
            cloudFeature &cur_cloud_feature = cur_feature_.second[IDX_REF];
            cloudFeature &prev_cloud_feature = prev_feature_.second[IDX_REF];
            pose_rlt_[IDX_REF] = lidar_tracker_.trackCloud(prev_cloud_feature, cur_cloud_feature, pose_rlt_[IDX_REF]);
            Pose pose_cur = Pose(Qs_.last(), Ts_.last()) * pose_rlt_[IDX_REF];
            std::cout << "relative transform: " << pose_rlt_[IDX_REF] << std::endl;
            std::cout << "current transform: " << pose_cur << std::endl;
            Qs_[cir_buf_cnt_] = pose_cur.q_;
            Ts_[cir_buf_cnt_] = pose_cur.t_;

            buildLocalMap();
            optimizeLocalMap();
            slideWindow();

            // lidar_optimizer_.OptimizeLocalMap(cur_feature_);
            // pose_world_laser_[];
            ROS_DEBUG("solver costs: %fms", t_solve.toc());
            break;
        }
    }

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
    // prev_feature_.second = cur_feature_.second;
}

void Estimator::optimizeLocalMap()
{
    

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
        for (size_t n = 0; n < NUM_OF_LASER; n++)
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
            printf("Laser_%d: initialize a local map size: %d\n", surf_points_stack_[n][pivot_idx].size());
        }
        ini_fixed_local_map_ = true;
    }

    // -----------------
    // build the whole local map using all poses
    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        surf_points_local_map_[n].clear();
        surf_points_local_map_filtered_[n].clear();
        // corner_points_local_map_[n].clear();
        // corner_points_local_map_filtered_[n].clear();
    }
    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        Pose pose_ext = Pose(qbl_[n], tbl_[n]);
        for (size_t i = pivot_idx; i < WINDOW_SIZE; i++)
        {
            Pose pose_i(Qs_[i], Ts_[i]);
            Eigen::Affine3d transform_pivot_i;
            transform_pivot_i.matrix() = (pose_pivot.T_ * pose_ext.T_).inverse() * (pose_i.T_ * pose_ext.T_);
            pcl::transformPointCloud(surf_points_stack_[n][i], surf_points_transformed, transform_pivot_i.cast<float>());
            for (auto &p: surf_points_transformed.points) p.intensity = i;
            surf_points_local_map_[n] += surf_points_transformed;
            // pcl::transformPointCloud(corner_points_stack_[n][idx], corner_points_transformed, transform_pivot_i);
            // for (auto &p: corner_points_transformed.points) p.intensity = 255.0 * i / WINDOW_SIZE;
            // corner_points_local_map_[n] += corner_points_transformed;
        }
        f_extract_.down_size_filter_surf_.setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_[n]));
        f_extract_.down_size_filter_surf_.filter(surf_points_local_map_filtered_[n]);
        // down_size_filter_corner_.setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_));
        // down_size_filter_corner_.filter(corner_points_local_map_filtered_);
    }
    printf("build local map: %fms\n", t_build_local_map.toc());

    // -----------------
    // calculate features
    TicToc t_local_map_extract;
    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_points_local_map(new pcl::KdTreeFLANN<PointI>());
        kdtree_surf_points_local_map.setInputCloud(boost::make_shared<PointICloud>(surf_points_local_map_filtered_[n]);
        // pcl::KdTreeFLANN<PointI>::Ptr kdtree_corner_points_local_map(new pcl::KdTreeFLANN<PointI>());
        // kdtree_corner_points_local_map.setInputCloud(boost::make_shared<PointICloud>(corner_points_local_map_filtered_[n]);
        for (size_t i = pivot_idx; i < WINDOW_SIZE; i++)
        {
            if (i != WINDOW_SIZE)
            {
                f_extract_.extractSurfFromMap(kdtree_surf_points_local_map, surf_points_local_map_filtered_[n],
                                            surf_points_stack_[n][i], features);
            } else
            {
                // calculateDistance(surf_points_local_map_filtered_[n], surf_points_stack_[n][idx]);
            }

        }
    }
    printf("extract local map: %fms\n", t_local_map_extract.toc());
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
        for (size_t n = 0; n < NUM_OF_LASER; n++)
        {
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
    Qs_.push(Qs_.last());
    Ts_.push(Ts_.last());
    Header_.push(Header_.last());
    // std::cout << "pose: " << Pose(Qs_.last(), Ts_.last()) << std::endl;
    for (size_t n = 0; n < NUM_OF_LASER; n++)
    {
        // printf("Laser: %d, current surf size: %d\n", n, surf_points_stack_size_[n].last());
        // printf("Laser: %d, localmap size: %d\n", n, surf_points_local_map_[n].size());
        surf_points_stack_[n].push(surf_points_stack_[n].last());
        surf_points_stack_size_[n].push(surf_points_stack_size_[n].last());
    }
    printf("slide window: %fms\n", t_solid_window.toc());
}

//
