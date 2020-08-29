/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "mloam_loop/pose_graph.h"

PoseGraph::PoseGraph()
{
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 0.0, 1.0);
    posegraph_visualization->setScale(0.1);
    posegraph_visualization->setLineWidth(0.1);
    
    skip_cnt_ = 0;
    earliest_loop_index_ = -1;
    global_index_ = 0;
    pgo_flag_ = false;

    laser_cloud_surf_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_corner_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_surf_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_corner_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());    
    laser_cloud_surf_from_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_corner_from_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_surf_from_map_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_corner_from_map_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    down_size_filter_surf_map_.setLeafSize(0.4, 0.4, 0.4);
    down_size_filter_corner_map_.setLeafSize(0.4, 0.4, 0.4);
    // down_size_filter_surf_map_.setLeafSize(1.0, 1.0, 1.0);
    // down_size_filter_corner_map_.setLeafSize(1.0, 1.0, 1.0);    
    kdtree_surf_from_map_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_corner_from_map_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
}

PoseGraph::~PoseGraph()
{
    t_optimization.detach();
}

void PoseGraph::registerPub(ros::NodeHandle &nh)
{
    pub_pg_path_ = nh.advertise<nav_msgs::Path>("/pose_graph_path", 1000);
    pub_pose_graph_ = nh.advertise<visualization_msgs::MarkerArray>("/pose_graph", 1000);
    pub_sc_ = nh.advertise<sensor_msgs::Image>("/scan_context", 5);
    pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/kf_cloud", 5);
    pub_loop_map_ = nh.advertise<sensor_msgs::PointCloud2>("/loop_map", 5);
    pub_loop_info_ = nh.advertise<mloam_msgs::Keyframes>("/loop_info", 5);
}

void PoseGraph::setParameter()
{
    sc_manager_.setParameter(LIDAR_HEIGHT,
                             PC_NUM_RING,
                             PC_NUM_SECTOR,
                             PC_MAX_RADIUS,
                             PC_UNIT_SECTORANGLE,
                             PC_UNIT_RINGGAP,
                             NUM_EXCLUDE_RECENT,
                             NUM_CANDIDATES_FROM_TREE,
                             SEARCH_RATIO,
                             SC_DIST_THRES,
                             TREE_MAKING_PERIOD);
}

void PoseGraph::setPGOTread()
{
    printf("[PoseGraph] set new pose graph thread, perfrom 6 DoF pose graph optimization\n");
    t_optimization = std::thread(&PoseGraph::optimizePoseGraph, this);
}

void PoseGraph::addKeyFrameIntoDB(KeyFrame *keyframe)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *raw_cloud += *keyframe->full_cloud_;
    *raw_cloud += *keyframe->outlier_cloud_;
    sc_manager_.makeAndSaveScancontextAndKeys(*raw_cloud);
    // if (VISUALIZE_IMAGE)
    // {
    //     cv::Mat tmp1_image = sc_manager_.getScanContextImage(que_index);
    //     cv::Mat tmp2_image = sc_manager_.getScanContextImage(detect);
    //     putText(tmp2_image, "loop score:" + to_string(qr.score_), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
    //     cv::vconcat(loop_result, tmp1_image, tmp2_image);
    // }
}

void PoseGraph::addKeyFrame(KeyFrame *cur_kf, bool flag_detect_loop)
{
    cur_kf->index_ = global_index_;
    global_index_++;
    keyframelist_.push_back(cur_kf);
    if (flag_detect_loop)
    {
        // detect loop candidates
        TicToc t_loop_detect;
        std::pair<int, double> ld_result = detectLoop(cur_kf, cur_kf->index_);
        printf("loop_detection: %fms\n", t_loop_detect.toc());
        if (ld_result.first != -1)
        {           
            int loop_index = ld_result.first;
            double yaw_diff_rad = ld_result.second;
            printf("find a loop candidate: %d <-> %d, yaw_ini: %f\n", cur_kf->index_, loop_index, yaw_diff_rad);

            // check temporal consistency
            TicToc t_check_tc;
            std::pair<bool, int> tc_result = checkTemporalConsistency(cur_kf->index_, loop_index);
            printf("check temporal consistency: %fms\n", t_check_tc.toc());
            if (!tc_result.first)
            {
                printf("loop reject with temporal verificiation\n");
            }
            else 
            {
                skip_cnt_ = 0; // not perform frequent geometric verification

                // set the initial guess using the yaw
                Eigen::Quaterniond q_ini(Eigen::AngleAxisd(yaw_diff_rad, Eigen::Vector3d::UnitZ()));
                Eigen::Vector3d t_ini = Eigen::Vector3d::Zero();
                Pose pose_ini_map_kf(q_ini, t_ini);

                // check geometric consistency
                TicToc t_check_gc;
                std::pair<bool, Pose> reg_result = checkGeometricConsistency(cur_kf, cur_kf->index_,
                                                                             loop_index,
                                                                             pose_ini_map_kf);
                printf("check geoometryc consistency %fs\n", t_check_gc.toc() / 1000);
                if (!reg_result.first)
                {
                    printf("loop reject with geometry verificiation\n");
                }
                else 
                {
                    // perform pose graph optimization
                    Pose loop_info = reg_result.second;
                    cur_kf->updateLoopInfo(loop_index, loop_info);
                    if (earliest_loop_index_ > loop_index || earliest_loop_index_ == -1)
                        earliest_loop_index_ = loop_index;

                    m_optimize_buf.lock();
                    optimize_buf_.push(cur_kf->index_);
                    m_optimize_buf.unlock();
                }
            }
        }
    }
    else
    {
        // for not frequently performing loop closure
        addKeyFrameIntoDB(cur_kf);
    }

    m_keyframelist.lock();

    Pose pose_w;
    cur_kf->getPose(pose_w);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp_);
    pose_stamped.header.frame_id = "/world";
    pose_stamped.pose.position.x = pose_w.t_(0) + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = pose_w.t_(1) + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = pose_w.t_(2);
    pose_stamped.pose.orientation.x = pose_w.q_.x();
    pose_stamped.pose.orientation.y = pose_w.q_.y();
    pose_stamped.pose.orientation.z = pose_w.q_.z();
    pose_stamped.pose.orientation.w = pose_w.q_.w();
    pg_path_.poses.push_back(pose_stamped);
    pg_path_.header = pose_stamped.header;
    posegraph_visualization->add_lidar_pose(pose_w.t_, pose_w.q_);

    if (RESULT_SAVE)
    {
        ofstream loop_path_file(MLOAM_LOOP_PATH, ios::app);
        loop_path_file.setf(ios::fixed, ios::floatfield);
        loop_path_file.precision(15);
        loop_path_file << cur_kf->time_stamp_ << " ";
        loop_path_file.precision(8);
        loop_path_file << pose_w.t_[0] << " "
                       << pose_w.t_[1] << " "
                       << pose_w.t_[2] << " "
                       << pose_w.q_.x() << " "
                       << pose_w.q_.y() << " "
                       << pose_w.q_.z() << " "
                       << pose_w.q_.w() << " " << std::endl;
        loop_path_file.close();
    }

    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame *>::reverse_iterator rit = keyframelist_.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist_.rend())
                break;
            Pose connected_pose;
            (*rit)->getPose(connected_pose);
            posegraph_visualization->add_edge(pose_w.t_, connected_pose.t_);
            rit++;
        }
    }    
    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop_)
        {
            //printf("has loop \n");
            KeyFrame *connected_KF = getKeyFrame(cur_kf->loop_index_);
            Pose connected_pose; 
            connected_KF->getPose(connected_pose);
            Pose pose_0;
            cur_kf->getPose(pose_0);
            //printf("add loop into visual \n");
            posegraph_visualization->add_loopedge(pose_0.t_, connected_pose.t_ + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
        }
    }

    publish();
    m_keyframelist.unlock();
}

void PoseGraph::loadKeyFrame(KeyFrame *cur_kf, bool flag_detect_loop)
{
    cur_kf->index_ = global_index_;
    global_index_++;
    int loop_index = -1;
    addKeyFrameIntoDB(cur_kf);

    m_keyframelist.lock();

    Pose pose_w;
    cur_kf->getPose(pose_w);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp_);
    pose_stamped.header.frame_id = "/world";
    pose_stamped.pose.position.x = pose_w.t_(0) + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = pose_w.t_(1) + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = pose_w.t_(2);
    pose_stamped.pose.orientation.x = pose_w.q_.x();
    pose_stamped.pose.orientation.y = pose_w.q_.y();
    pose_stamped.pose.orientation.z = pose_w.q_.z();
    pose_stamped.pose.orientation.w = pose_w.q_.w();
    pg_path_.poses.push_back(pose_stamped);
    pg_path_.header = pose_stamped.header;
    posegraph_visualization->add_lidar_pose(pose_w.t_, pose_w.q_);

    if (SHOW_S_EDGE)
    {
        list<KeyFrame *>::reverse_iterator rit = keyframelist_.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist_.rend())
                break;
            Pose connected_pose;
            (*rit)->getPose(connected_pose);
            posegraph_visualization->add_edge(pose_w.t_, connected_pose.t_);
            rit++;
        }
    }

    /*
    if (cur_kf->has_loop)
    {
        //printf("has loop \n");
        KeyFrame *connected_KF = getKeyFrame(cur_kf->loop_index);
        Pose connected_pose; 
        connected_KF->getPose(connected_pose);
        //printf("add loop into visual \n");
        posegraph_visualization->add_loopedge(pose_w.t_, connected_pose.t_ + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
    }
    */

    keyframelist_.push_back(cur_kf);
    publish();
    m_keyframelist.unlock();
}

std::pair<int, double> PoseGraph::detectLoop(const KeyFrame *keyframe, const int que_index)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    *raw_cloud += *keyframe->full_cloud_;
    *raw_cloud += *keyframe->outlier_cloud_;
    sc_manager_.makeAndSaveScancontextAndKeys(*raw_cloud);
    assert(que_index < 0 || que_index >= sc_manager_.getDataBaseSize());

    // apply scan context-based global localization
    QueryResult qr = sc_manager_.detectLoopClosureID(que_index);
    std::cout << qr << std::endl;
    std::pair<int, double> detect_result;
    detect_result.first = qr.match_index_;
    detect_result.second = qr.yaw_diff_rad_;
    cv::Mat loop_result;
    if (detect_result.first != -1)
    {
        int match_index = detect_result.first;
        Eigen::Vector3d t_que = keyframe->pose_w_.t_;
        Eigen::Vector3d t_match;
        list<KeyFrame *>::iterator it;
        for (it = keyframelist_.begin(); it != keyframelist_.end(); it++)
        {
            if ((*it)->index_ == match_index)
            {
                t_match = (*it)->pose_w_.t_;
                break;
            }
        }
        // check if the candidate loop is to far
        if ((t_que - t_match).norm() > LOOP_DISTANCE_THRESHOLD)
        {
            printf("loop reject since distance is far: %f\n", (t_que - t_match).norm());
            detect_result.first = -1;
        }
        // if (VISUALIZE_IMAGE)
        // {
        //     cv::Mat tmp1_image = sc_manager_.getScanContextImage(que_index);
        //     cv::Mat tmp2_image = sc_manager_.getScanContextImage(match_index);
        //     putText(tmp2_image, "loop score:" + to_string(qr.score_), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        //     loop_result = tmp1_image.clone();
        //     cv::vconcat(loop_result, tmp2_image, loop_result);
        //     cv::imshow("loop_result", loop_result);
        //     cv::waitKey(20);
        // }
    }
    return detect_result;
}

std::pair<bool, int> PoseGraph::checkTemporalConsistency(const int &que_index,
                                                         const int &match_index)
{
    bool tc_flag = true;
    int cnt = 0;
    // std::vector<int> all_match_index;
    // all_match_index.push_back(match_index);
    // for (int reque_index = que_index - 5; reque_index < que_index; reque_index++)
    // {
    //     QueryResult qr = sc_manager_.detectLoopClosureID(reque_index);
    //     std::pair<int, double> redetect_result;
    //     redetect_result.first = qr.match_index_;
    //     redetect_result.second = qr.yaw_diff_rad_;
    //     int rematch_index = redetect_result.first;
    //     // std::cout << "reque_index: " << reque_index << std::endl;
    //     if ((rematch_index == -1) || (abs(match_index - rematch_index) > LOOP_TEMPORAL_CONSISTENCY_THRESHOLD))
    //     {
    //         tc_flag = false;
    //         printf("%d <-> %d\n", match_index, rematch_index);
    //         printf("loop reject since only find %d matchings\n", cnt);
    //         break;
    //     }
    //     cnt++;
    //     // all_match_index.push_back(rematch_index);
    // }
    // if (tc_flag)
    // {
    //     for (size_t i = 0; i < all_match_index.size(); i++)
    //         std::cout << "que_index: " << que_index - i << ", matcded: " << all_match_index[i] << std::endl;
    // }
    return make_pair(tc_flag, match_index);
}

// all point clouds are transformed into the map (local) frame
void PoseGraph::constructLocalMap(const KeyFrame *cur_kf,
                                  const int &que_index,
                                  const int &match_index,
                                  const Pose &pose_ini)
{
    pcl::PointCloud<pcl::PointXYZI> surf_trans, corner_trans;

    // construct the keyframe point cloud
    laser_cloud_surf_->clear();
    laser_cloud_corner_->clear();
    for (int j = -LOOP_HISTORY_SEARCH_NUM; j <= 0; j++)
    {
        if (que_index + j < 0)
            continue;
        KeyFrame *tmp_kf = getKeyFrame(que_index + j);
        if (!tmp_kf)
            continue;
        Eigen::Matrix4d T_relative = cur_kf->pose_w_.T_.inverse() * tmp_kf->pose_w_.T_;
        Eigen::Matrix4d T_ini_map_kf = pose_ini.T_ * T_relative;
        pcl::transformPointCloud(*tmp_kf->surf_cloud_, surf_trans, T_ini_map_kf.cast<float>());
        *laser_cloud_surf_ += surf_trans;
        pcl::transformPointCloud(*tmp_kf->corner_cloud_, corner_trans, T_ini_map_kf.cast<float>());
        *laser_cloud_corner_ += corner_trans;
    }
    down_size_filter_surf_map_.setInputCloud(laser_cloud_surf_);
    down_size_filter_surf_map_.filter(*laser_cloud_surf_ds_);
    down_size_filter_corner_map_.setInputCloud(laser_cloud_corner_);
    down_size_filter_corner_map_.filter(*laser_cloud_corner_ds_);
    printf("[loop_closure] kf surf num: %lu, corner num: %lu\n", laser_cloud_surf_ds_->size(), laser_cloud_corner_ds_->size());

    // construct the model point cloud
    laser_cloud_surf_from_map_->clear();
    laser_cloud_corner_from_map_->clear();
    KeyFrame *old_kf = getKeyFrame(match_index); // check NULL
    for (int j = -LOOP_HISTORY_SEARCH_NUM; j <= LOOP_HISTORY_SEARCH_NUM; j++)
    {
        if (match_index + j < 0 || match_index + j >= que_index)
            continue;
        KeyFrame *tmp_kf = getKeyFrame(match_index + j);
        if (!tmp_kf)
            continue;
        Eigen::Matrix4d T_relative = old_kf->pose_w_.T_.inverse() * tmp_kf->pose_w_.T_;
        pcl::transformPointCloud(*tmp_kf->surf_cloud_, surf_trans, T_relative.cast<float>());
        *laser_cloud_surf_from_map_ += surf_trans;
        pcl::transformPointCloud(*tmp_kf->corner_cloud_, corner_trans, T_relative.cast<float>());
        *laser_cloud_corner_from_map_ += corner_trans;
    }
    down_size_filter_surf_map_.setInputCloud(laser_cloud_surf_from_map_);
    down_size_filter_surf_map_.filter(*laser_cloud_surf_from_map_ds_);
    down_size_filter_corner_map_.setInputCloud(laser_cloud_corner_from_map_);
    down_size_filter_corner_map_.filter(*laser_cloud_corner_from_map_ds_);

    size_t laser_cloud_surf_from_map_num = laser_cloud_surf_from_map_ds_->size();
    size_t laser_cloud_corner_from_map_num = laser_cloud_corner_from_map_ds_->size();
    printf("[loop_closure] map surf num: %lu, corner num: %lu\n", laser_cloud_surf_from_map_num, laser_cloud_corner_from_map_num);
}

std::pair<bool, Pose> PoseGraph::checkGeometricConsistency(const KeyFrame *cur_kf,
                                                           const int &que_index,
                                                           const int &match_index,
                                                           const Pose &pose_ini)
{
    assert(que_index < 0);
    assert(match_index < 0);

    // map constrcution: give initial transformation on the kf
    TicToc t_map_construction;
    constructLocalMap(cur_kf, que_index, match_index, pose_ini);
    printf("[loop_closure] map construction: %fms\n", t_map_construction.toc()); // 47ms

    // global registration: initial guess is identity
    TicToc t_global_reg;
    std::pair<bool, Eigen::Matrix4d> global_reg_result =
        loop_reg_.performGlobalRegistration(laser_cloud_surf_from_map_ds_,
                                            laser_cloud_surf_ds_);
    printf("global registration: %fs\n", t_global_reg.toc() / 1000);
    Pose pose_global(global_reg_result.second.cast<double>());
    if (!global_reg_result.first)
    {
        printf("loop reject in global registration ...\n");
        return make_pair(false, pose_global);
    }

    // lobal registration: initial guess is the result of global registration
    TicToc t_local_reg;
    std::pair<bool, Eigen::Matrix4d> local_reg_result =
        loop_reg_.performLocalRegistration(laser_cloud_surf_from_map_ds_,
                                           laser_cloud_corner_from_map_ds_,
                                           laser_cloud_surf_ds_,
                                           laser_cloud_corner_ds_,
                                           global_reg_result.second);
    printf("local registration: %fs\n", t_local_reg.toc() / 1000);
    Pose pose_icp(local_reg_result.second * pose_ini.T_);
    if (!local_reg_result.first)
    {
        printf("loop reject in local registration ...\n");
        return make_pair(false, pose_icp);
    }
    std::cout << "Find loop, relative transformation: " << pose_icp << std::endl;

    if (LOOP_SAVE_PCD)
    {
        pcl::PointCloud<pcl::PointXYZI> surf_trans, corner_trans;
        pcl::transformPointCloud(*laser_cloud_surf_ds_, surf_trans, local_reg_result.second.cast<float>());
        // pcl::transformPointCloud(*laser_cloud_corner_ds_, corner_trans, local_reg_result.second.cast<float>());
        pcd_writer_.write(POSE_GRAPH_SAVE_PATH + to_string(que_index) + "_data.pcd", *laser_cloud_surf_ds_);
        pcd_writer_.write(POSE_GRAPH_SAVE_PATH + to_string(que_index) + "_data_icp.pcd", surf_trans);
        pcd_writer_.write(POSE_GRAPH_SAVE_PATH + to_string(que_index) + "_model.pcd", *laser_cloud_surf_from_map_ds_);
    }
    return make_pair(true, pose_icp);
}

KeyFrame *PoseGraph::getKeyFrame(int index)
{
    //    unique_lock<mutex> lock(m_keyframelist_);
    list<KeyFrame *>::iterator it = keyframelist_.begin();
    for (; it != keyframelist_.end(); it++)
    {
        if ((*it)->index_ == index)
            break;
    }
    if (it != keyframelist_.end())
        return *it;
    else
        return NULL;
}

void PoseGraph::optimizePoseGraph()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while (!optimize_buf_.empty())
        {
            cur_index = optimize_buf_.front();
            first_looped_index = earliest_loop_index_;
            optimize_buf_.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            printf("optimize pose graph \n");
            TicToc t_pgo;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);
            int max_length = cur_index + 1;
            double t_array[max_length][3];
            double q_array[max_length][4];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //ptions.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            // loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            list<KeyFrame*>::iterator it;
            int i = 0; // the index of the array
            for (it = keyframelist_.begin(); it != keyframelist_.end(); it++)
            {
                if ((*it)->index_ < first_looped_index)
                    continue;
                (*it)->local_index_ = i;
                Pose tmp_pose;
                (*it)->getPose(tmp_pose);
                t_array[i][0] = tmp_pose.t_(0);
                t_array[i][1] = tmp_pose.t_(1);
                t_array[i][2] = tmp_pose.t_(2);
                q_array[i][0] = tmp_pose.q_.w();
                q_array[i][1] = tmp_pose.q_.x();
                q_array[i][2] = tmp_pose.q_.y();
                q_array[i][3] = tmp_pose.q_.z();
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
                if ((*it)->index_ == first_looped_index) 
                {   
                    problem.SetParameterBlockConstant(q_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                // add edge between previous frames
                for (int j = 1; j < 5; j++)
                {
                    if (i - j >= 0)
                    {
                        Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                        Quaterniond q_i_j = Quaterniond(q_array[i-j][0], q_array[i-j][1], q_array[i-j][2], q_array[i-j][3]);
                        Quaterniond q_i = Quaterniond(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                        relative_t = q_i_j.inverse() * relative_t;
                        Quaterniond relative_q = q_i_j.inverse() * q_i;
                        ceres::CostFunction *f = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                         relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                         0.1, 0.01);
                        problem.AddResidualBlock(f, NULL, q_array[i-j], t_array[i-j], q_array[i], t_array[i]);
                    }
                }

                // add loop edge
                if((*it)->has_loop_)
                {
                    assert((*it)->loop_index_ >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index_)->local_index_;
                    Pose pose_relative = (*it)->getLoopRelativePose();
                    Eigen::Vector3d relative_t = pose_relative.t_;
                    Eigen::Quaterniond relative_q = pose_relative.q_;
                    ceres::CostFunction *loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                 0.1, 0.01);
                    problem.AddResidualBlock(loop_function, loss_function, q_array[connected_index], t_array[connected_index], q_array[i], t_array[i]);                    

                    {
                        Pose Twci, Twcj;
                        (*it)->getPose(Twci);
                        getKeyFrame((*it)->loop_index_)->getPose(Twcj);
                        Pose rel_pose = Twcj.inverse() * Twci;
                        std::cout << "est map-kf rel pose: " << rel_pose << std::endl;

                        Pose pose_relative;
                        pose_relative = (*it)->getLoopRelativePose();
                        std::cout << "loop map-kf rel pose: " << pose_relative << std::endl;
                        std::cout << "they should be equal" << std::endl;
                    }

                }
                if ((*it)->index_ == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << "\n";
            //printf("pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();

            i = 0;
            for (it = keyframelist_.begin(); it != keyframelist_.end(); it++)
            {
                if ((*it)->index_ < first_looped_index)
                    continue;
                Quaterniond tmp_q(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Pose tmp_pose(tmp_q, tmp_t);
                (*it)->updatePose(tmp_pose);
                if ((*it)->index_ == cur_index)
                    break;
                i++;
            }

            // update the pose in behind frames
            Pose cur_pose_w, last_pose_w;
            cur_kf->getPose(cur_pose_w);
            cur_kf->getLastPose(last_pose_w);
            Pose pose_drift = cur_pose_w * last_pose_w.inverse();
            it++;
            for (; it != keyframelist_.end(); it++)
            {
                Pose update_pose;
                (*it)->getPose(update_pose);
                update_pose = pose_drift * update_pose;
                (*it)->updatePose(update_pose);
            }
            pgo_flag_ = true;

            m_keyframelist.unlock();
            updatePath();
            publishLoopInfo();
            printf("perform pose graph optimization: %fs\n", t_pgo.toc() / 1000);
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
    return;
}

void PoseGraph::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc t_save_pose_graph;
    FILE *pFile;
    printf("[PoseGraph] pose graph path: %s\n", POSE_GRAPH_SAVE_PATH.c_str());
    printf("[PoseGraph] pose graph saving %lu keyframes\n", keyframelist_.size());
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    pFile = fopen(file_path.c_str(),"w");
    // fprintf(pFile, "index time_stamp px py pz qx qy qz qw loop_index loop_info\n");
    list<KeyFrame*>::iterator it;
    for (it = keyframelist_.begin(); it != keyframelist_.end(); it++)
    {
        std::string pcd_path;
        if (LOOP_SAVE_PCD)
        {
            // printf("index: %lu, surf cloud size: %lu\n", (*it)->index_, (*it)->surf_cloud_->size());
            // pcd_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index_) + "_surf_cloud.pcd";
            // pcd_writer_.write(pcd_path, *(*it)->surf_cloud_);
            // pcd_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index_) + "_corner_cloud.pcd";
            // pcd_writer_.write(pcd_path, *(*it)->corner_cloud_);
            // pcd_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index_) + "_full_cloud.pcd";
            // pcd_writer_.write(pcd_path, *(*it)->full_cloud_);
            // pcd_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index_) + "_outlier_cloud.pcd";
            // pcd_writer_.write(pcd_path, *(*it)->outlier_cloud_);
        }
        Pose tmp_pose = (*it)->pose_w_;
        Pose loop_info = (*it)->loop_info_; // the relative pose
        fprintf(pFile, " %d %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f\n",
                (*it)->index_, (*it)->time_stamp_,
                tmp_pose.t_(0), tmp_pose.t_(1), tmp_pose.t_(2),
                tmp_pose.q_.x(), tmp_pose.q_.y(), tmp_pose.q_.z(), tmp_pose.q_.w(),
                (*it)->loop_index_,
                loop_info.t_(0), loop_info.t_(1), loop_info.t_(2),
                loop_info.q_.x(), loop_info.q_.y(), loop_info.q_.z(), loop_info.q_.w());
    }
    fclose(pFile);
    printf("[PoseGraph] save pose graph time: %fs\n", t_save_pose_graph.toc() / 1000);
    m_keyframelist.unlock();
}

void PoseGraph::loadPoseGraph()
{
    TicToc t_load_posegraph;
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("[PoseGraph] load pose graph from: %s \n", file_path.c_str());
    printf("[PoseGraph] pose graph loading...\n");
    pFile = fopen(file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("[PoseGraph] lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double pose_w_tx, pose_w_ty, pose_w_tz;
    double pose_w_qx, pose_w_qy, pose_w_qz, pose_w_qw;
    double loop_info_tx, loop_info_ty, loop_info_tz;
    double loop_info_qx, loop_info_qy, loop_info_qz, loop_info_qw;
    int loop_index;
    int cnt = 0;
    while (fscanf(pFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf",
                  &index, &time_stamp,
                  &pose_w_tx, &pose_w_ty, &pose_w_tz,
                  &pose_w_qx, &pose_w_qy, &pose_w_qz, &pose_w_qw,
                  &loop_index,
                  &loop_info_tx, &loop_info_ty, &loop_info_tz,
                  &loop_info_qx, &loop_info_qy, &loop_info_qz, &loop_info_qw) != EOF)
    {
        // printf("I read: %d %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f\n",
        //        index, time_stamp,
        //        pose_w_tx, pose_w_ty, pose_w_tz,
        //        pose_w_qx, pose_w_qy, pose_w_qz, pose_w_qw,
        //        loop_index,
        //        loop_info_tx, loop_info_ty, loop_info_tz,
        //        loop_info_qx, loop_info_qy, loop_info_qz, loop_info_qw);
        pcl::PointCloud<pcl::PointXYZI>::Ptr surf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        std::string pcd_path;
        if (LOOP_SAVE_PCD)
        {
            pcd_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_surf_cloud.pcd";
            pcd_reader_.read(pcd_path, *surf_cloud);
            pcd_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_corner_cloud.pcd";
            pcd_reader_.read(pcd_path, *corner_cloud);
            pcd_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_full_cloud.pcd";
            pcd_reader_.read(pcd_path, *full_cloud);
            pcd_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_outlier_cloud.pcd";
            pcd_reader_.read(pcd_path, *outlier_cloud);
        }
        Pose pose_w = Pose(Eigen::Quaterniond(pose_w_qw, pose_w_qx, pose_w_qy, pose_w_qz),
                           Eigen::Vector3d(pose_w_tx, pose_w_ty, pose_w_tz));

        Pose loop_info = Pose(Eigen::Quaterniond(loop_info_qw, loop_info_qx, loop_info_qy, loop_info_qz),
                              Eigen::Vector3d(loop_info_tx, loop_info_ty, loop_info_tz));
        
        if (loop_index != -1)
        {
            if (earliest_loop_index_ > loop_index || earliest_loop_index_ == -1)
            {
                earliest_loop_index_ = loop_index;
            }
        }

        KeyFrame *keyframe = new KeyFrame(time_stamp,
                                          index,
                                          pose_w,
                                          surf_cloud,
                                          corner_cloud,
                                          full_cloud,
                                          outlier_cloud,
                                          loop_index,
                                          loop_info,
                                          0);
        loadKeyFrame(keyframe, 0);
        if (cnt % 20 == 0)
        {
            publish();
        }
        cnt++;
    }
    fclose (pFile);
    printf("[PoseGraph] load pose graph time: %f s\n", t_load_posegraph.toc() / 1000);
}

void PoseGraph::updatePath()
{
    m_keyframelist.lock();
    list<KeyFrame *>::iterator it;
    pg_path_.poses.clear();
    posegraph_visualization->reset();

    if (RESULT_SAVE)
    {
        ofstream loop_path_file_tmp(MLOAM_LOOP_PATH, ios::out);
        loop_path_file_tmp.close();
    }

    for (it = keyframelist_.begin(); it != keyframelist_.end(); it++)
    {
        Pose pose_w;
        (*it)->getPose(pose_w);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time((*it)->time_stamp_);
        pose_stamped.header.frame_id = "/world";
        pose_stamped.pose.position.x = pose_w.t_(0) + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = pose_w.t_(1) + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = pose_w.t_(2);
        pose_stamped.pose.orientation.x = pose_w.q_.x();
        pose_stamped.pose.orientation.y = pose_w.q_.y();
        pose_stamped.pose.orientation.z = pose_w.q_.z();
        pose_stamped.pose.orientation.w = pose_w.q_.w();
        pg_path_.poses.push_back(pose_stamped);
        pg_path_.header = pose_stamped.header;
        posegraph_visualization->add_lidar_pose(pose_w.t_, pose_w.q_);

        if (RESULT_SAVE)
        {
            ofstream loop_path_file(MLOAM_LOOP_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            loop_path_file.precision(15);
            loop_path_file << (*it)->time_stamp_ << " ";
            loop_path_file.precision(8);
            loop_path_file << pose_w.t_[0] << " "
                           << pose_w.t_[2] << " "
                           << pose_w.t_[3] << " "
                           << pose_w.q_.x() << " "
                           << pose_w.q_.y() << " "
                           << pose_w.q_.z() << " "
                           << pose_w.q_.w() << " "
                           << endl;
            loop_path_file.close();
        }

        if (SHOW_S_EDGE)
        {
            list<KeyFrame *>::reverse_iterator rit = keyframelist_.rbegin();
            list<KeyFrame *>::reverse_iterator lrit;
            for (; rit != keyframelist_.rend(); rit++)
            {
                if ((*rit)->index_ == (*it)->index_)
                {
                    lrit = rit;
                    lrit++;
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist_.rend())
                            break;
                        Pose connected_pose;
                        (*lrit)->getPose(connected_pose);
                        posegraph_visualization->add_edge(pose_w.t_, connected_pose.t_);
                        lrit++;
                    }
                    break;
                }
            }
        }
        if (SHOW_L_EDGE)
        {
            if ((*it)->has_loop_)
            {
                KeyFrame *connected_KF = getKeyFrame((*it)->loop_index_);
                Pose connected_pose;
                connected_KF->getPose(connected_pose);
                Pose pose_0;
                (*it)->getPose(pose_0);
                posegraph_visualization->add_loopedge(pose_0.t_, connected_pose.t_ + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
            }
        }
    }
    publish();
    m_keyframelist.unlock();
}

void PoseGraph::publishLoopInfo()
{
    m_keyframelist.lock();
    mloam_msgs::Keyframes kf_path;
    list<KeyFrame *>::iterator it;
    for (it = keyframelist_.begin(); it != keyframelist_.end(); it++)
    {
        Pose pose_w;
        (*it)->getPose(pose_w);
        geometry_msgs::PoseWithCovarianceStamped pose_stamped_cov;
        pose_stamped_cov.header.stamp = ros::Time().fromSec((*it)->time_stamp_);
        pose_stamped_cov.header.frame_id = "/world";
        pose_stamped_cov.pose.pose.position.x = pose_w.t_.x();
        pose_stamped_cov.pose.pose.position.y = pose_w.t_.y();
        pose_stamped_cov.pose.pose.position.z = pose_w.t_.z();
        pose_stamped_cov.pose.pose.orientation.x = pose_w.q_.x();
        pose_stamped_cov.pose.pose.orientation.y = pose_w.q_.y();
        pose_stamped_cov.pose.pose.orientation.z = pose_w.q_.z();
        pose_stamped_cov.pose.pose.orientation.w = pose_w.q_.w();
        for (size_t i = 0; i < 6; i++)
            for (size_t j = 0; j < 6; j++)
                pose_stamped_cov.pose.covariance[i * 6 + j] = float(pose_w.cov_(i, j));
        kf_path.poses.push_back(pose_stamped_cov);
        kf_path.header = pose_stamped_cov.header;
    }
    if (pgo_flag_)
    {
        kf_path.status = 1;
    } 
    else
    {
        kf_path.status = 0;
    }
    pgo_flag_ = false;
    pub_loop_info_.publish(kf_path);
    printf("publish loop info\n");
    m_keyframelist.unlock();
}

void PoseGraph::publish()
{
    pub_pg_path_.publish(pg_path_);
    posegraph_visualization->publish_by(pub_pose_graph_, pg_path_.header);

    if (VISUALIZE_IMAGE)
    {
        KeyFrame *keyframe = keyframelist_.back();
        cv::Mat sc_img = sc_manager_.getScanContextImage(keyframe->index_);
        cv_bridge::CvImage sc_msg;
        sc_msg.header.frame_id = "/world";
        sc_msg.header.stamp = ros::Time().fromSec(keyframe->time_stamp_);
        sc_msg.encoding = sensor_msgs::image_encodings::RGB8;
        sc_msg.image = sc_img;
        pub_sc_.publish(sc_msg.toImageMsg());

        sensor_msgs::PointCloud2 msg_cloud;
        msg_cloud.header.frame_id = "/world";
        msg_cloud.header.stamp = ros::Time().fromSec(keyframe->time_stamp_);   
        pcl::toROSMsg(*laser_cloud_surf_, msg_cloud);
        pub_cloud_.publish(msg_cloud);
        pcl::toROSMsg(*laser_cloud_surf_from_map_ds_, msg_cloud);
        pub_loop_map_.publish(msg_cloud);
    }
}


int PoseGraph::getKeyFrameSize()
{
    return keyframelist_.size();
}