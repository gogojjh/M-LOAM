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
    frame_cnt_ = 0;

    td_ = 0;

    solver_flag_ = INITIAL;

    pose_laser_cur_.clear();
    pose_prev_cur_.clear();
    // pose_ext_.clear();
    calib_base_laser_.clear();

    initial_extrinsics_.clearState();

    m_process_.unlock();
}

void Estimator::setParameter()
{
    m_process_.lock();

    pose_laser_cur_.resize(NUM_OF_LASER);
    pose_prev_cur_.resize(NUM_OF_LASER);
    calib_base_laser_.resize(NUM_OF_LASER);
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        calib_base_laser_[i] = Pose(QBL[i], TBL[i], TDBL[i]);
        cout << " given extrinsic base_to_laser_" << i << ": " << calib_base_laser_[i] << endl;
    }

    initial_extrinsics_.setParameter();

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
    frame_cnt_++;
    printf("%d \n", frame_cnt_);

    // TODO: to parallize it?
    m_buf_.lock();
    std::vector<cloudFeature> feature_frame;
    TicToc feature_ext_time;
    for (size_t i = 0; i < v_laser_cloud_in.size(); i++)
    {
        printf("[LASER %d]: \n", i);
        feature_frame.push_back(f_extract_.extractCloud(t, v_laser_cloud_in[i]));
    }
    printf("featureExt time: %f ms \n", feature_ext_time.toc());
    feature_buf_.push(make_pair(t, feature_frame));
    m_buf_.unlock();

    TicToc process_time;
    processMeasurements();
    ROS_WARN_STREAM("processMea time: " << process_time.toc() << "ms");
}

void Estimator::inputCloud(const double &t,
    const PointCloud &laser_cloud_in0)
{
    frame_cnt_++;
    printf("%d \n", frame_cnt_);

    m_buf_.lock();
    std::vector<cloudFeature> feature_frame;
    TicToc feature_ext_time;
    printf("LASER 0: \n");
    feature_frame.push_back(f_extract_.extractCloud(t, laser_cloud_in0));
    printf("featureExt time: %f ms \n", feature_ext_time.toc());

    feature_buf_.push(make_pair(t, feature_frame));
    m_buf_.unlock();

    TicToc process_time;
    processMeasurements();
    ROS_WARN_STREAM("frame: " << frame_cnt_ << " ,processMea time: " << process_time.toc() << "ms");
}

void Estimator::processMeasurements()
{
    while (1)
    {
        printf("process measurments *********\n");
        if (!feature_buf_.empty())
        {
            cur_feature_ = feature_buf_.front();
            cur_time_ = cur_feature_.first + td_;
            assert(cur_feature_.second.size() == NUM_OF_LASER);

            m_buf_.lock();
            feature_buf_.pop();
            m_buf_.unlock();

            // -----------------
            // feature tracker: estimate the relative transformations of each lidar
            m_process_.lock();
            TicToc t_mloam_tracker;
            if (!b_system_inited_)
            {
                b_system_inited_ = true;
                printf("System initialization finished \n");
                for (size_t i = 0; i < NUM_OF_LASER; i++)
                {
                    pose_prev_cur_[i].push_back(Pose());
                    pose_laser_cur_[i].push_back(Pose());
                }
            } else
            {
                for (size_t i = 0; i < NUM_OF_LASER; i++)
                {
                    printf("[LASER %d]:\n", i);
                    cloudFeature &cur_cloud_feature = cur_feature_.second[i];
                    cloudFeature &prev_cloud_feature = prev_feature_.second[i];
                    Pose pose_rlt = lidar_tracker_.trackCloud(prev_cloud_feature, cur_cloud_feature, *(pose_prev_cur_[i].end()-1));
                    pose_prev_cur_[i].push_back(pose_rlt);
                    pose_laser_cur_[i].push_back(*(pose_laser_cur_[i].end()-1) * pose_rlt);

                    std::cout << "relative transform: " << pose_rlt << std::endl;
                    std::cout << "current transform: " << *(pose_laser_cur_[i].end()-1) << std::endl;
                }
                printf("mloam_tracker %f ms\n", t_mloam_tracker.toc());
            }

            prev_time_ = cur_time_;
            prev_feature_.first = prev_time_;
            // prev_feature_.second = cur_feature_.second;
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

            // -----------------
            // initialize extrinsics
            if (ESTIMATE_EXTRINSIC == 2)
            {
                ROS_INFO("calibrating extrinsic param, rotation movement is needed");
                TicToc t_calib_ext;
                if (frame_cnt_ != 0)
                {
                    for (size_t i = 0; i < NUM_OF_LASER; i++)
                    {
                        Pose calib_result;
                        if ((!initial_extrinsics_.cov_rot_state_[i]) &&
                            (initial_extrinsics_.calibExRotation(pose_prev_cur_[0], pose_prev_cur_[i], i, calib_result)))
                        {
                            initial_extrinsics_.setCovRotation(i);
                            ROS_WARN_STREAM("number of poses: " << frame_cnt_);
                            ROS_WARN_STREAM("initial extrinsic of laser_" << i << ": " << calib_result);
                            calib_base_laser_[i] = Pose(calib_result.q_, calib_result.t_, calib_result.td_);
                            QBL[i] = calib_result.q_;
                            TBL[i] = calib_result.t_;
                            TDBL[i] = calib_result.td_;
                        }
                    }
                    if (initial_extrinsics_.full_cov_rot_state_)
                    {
                        ROS_WARN("all initial extrinsic rotation calib success");
                        initial_extrinsics_.saveStatistics(pose_prev_cur_);
                        ESTIMATE_EXTRINSIC = 1;
                        solver_flag_ = NON_LINEAR;
                    }
                }
                printf("whole initialize extrinsics %fms\n", t_calib_ext.toc());
            }

            // -----------------
            // nonlinear optimization
            if (solver_flag_ == INITIAL)
            {

            } else if (solver_flag_ == NON_LINEAR)
            {
                TicToc t_solve;
                ROS_DEBUG("solver costs: %fms", t_solve.toc());
            }

            // -----------------
            // print and publish current result
            printStatistics(*this, 0);

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(cur_feature_.first);

            // printf("[estimator] publish point cloud\n");
            pubPointCloud(*this, header);

            // printf("[estimator] publis pose\n");
            pubOdometry(*this, header);

            // pubKeyPoses(*this, header);
            // pubCameraPose(*this, header);

            // pubKeyframe(*this);
            // pubTF(*this, header);

            m_process_.unlock();
        }

        if (!MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}



//
