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

#include "lidar_tracker.h"
#include "../factor/lidar_factor.hpp"

using namespace common;

#define DISTORTION 0

LidarTracker::LidarTracker()
{
    ROS_INFO("Tracker begin");
}

// undistort lidar point
void LidarTracker::TransformToStart(PointI const *const pi, PointI *const po, const Pose &pose)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, pose.q_);
    Eigen::Vector3d t_point_last = s * pose.t_;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame
void LidarTracker::TransformToEnd(PointI const *const pi, PointI *const po, const Pose &pose)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp, pose);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = pose.q_.inverse() * (un_point - pose.t_);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}

Pose LidarTracker::trackCloud(const cloudFeature &prev_cloud_feature,
    const cloudFeature &cur_cloud_feature,
    const Pose &pose_ini)
{
    TicToc t_whole;

    //-----------------
    // step 1: prev feature and kdtree
    PointICloudPtr corner_points_last(new PointICloud());
    PointICloudPtr surf_points_last(new PointICloud());
    pcl::KdTreeFLANN<PointI>::Ptr kdtree_corner_last(new pcl::KdTreeFLANN<PointI>());
    pcl::KdTreeFLANN<PointI>::Ptr kdtree_surf_last(new pcl::KdTreeFLANN<PointI>());

    *corner_points_last = prev_cloud_feature.find("corner_points_less_sharp")->second;
    *surf_points_last = prev_cloud_feature.find("surf_points_less_flat")->second;
    int corner_points_last_num = corner_points_last->points.size();
    int surf_points_last_num = surf_points_last->points.size();
    kdtree_corner_last->setInputCloud(corner_points_last);
    kdtree_surf_last->setInputCloud(surf_points_last);

    // step 2: current feature
    // PointICloud laser_cloud = cur_cloud_feature["laser_cloud"];
    PointICloudPtr corner_points_sharp(new PointICloud());
    PointICloudPtr surf_points_flat(new PointICloud());

    *corner_points_sharp = cur_cloud_feature.find("corner_points_sharp")->second;
    // PointICloud corner_points_less_sharp = cur_cloud_feature["corner_points_less_sharp"];
    *surf_points_flat = cur_cloud_feature.find("surf_points_flat")->second;
    // PointICloud surf_points_less_flat = cur_cloud_feature["surf_points_less_flat"];

    int corner_points_sharp_num = corner_points_sharp->points.size();
    int surf_points_sharp_num = surf_points_flat->points.size();

    double para_q[4] = {pose_ini.q_.x(), pose_ini.q_.y(), pose_ini.q_.z(), pose_ini.q_.w()};
    double para_t[3] = {pose_ini.t_(0), pose_ini.t_(1), pose_ini.t_(2)};

    Eigen::Map<Eigen::Quaterniond> q_prev_cur(para_q);
    Eigen::Map<Eigen::Vector3d> t_prev_cur(para_t);

    //-----------------
    int num_corner_correspondence;
    int num_plane_correspondence;
    int skip_frame_num = 5;

    TicToc t_opt;
    for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
    {
        num_corner_correspondence = 0;
        num_plane_correspondence = 0;

        // -----------------
        // step 1: set ceres solver
        //ceres::LossFunction *loss_function = NULL;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(para_q, 4, q_parameterization);
        problem.AddParameterBlock(para_t, 3);

        // -----------------
        // step 2: find correspondence for cloud features
        PointI point_sel;
        std::vector<int> point_search_ind;
        std::vector<float> point_search_sqdis;
        TicToc t_data;

        // find correspondence for corner features
        for (int i = 0; i < corner_points_sharp_num; ++i)
        {
            TransformToStart(&(corner_points_sharp->points[i]), &point_sel, Pose(q_prev_cur, t_prev_cur));
            kdtree_corner_last->nearestKSearch(point_sel, 1, point_search_ind, point_search_sqdis);

            int closest_point_ind = -1, min_point_ind2 = -1;
            if (point_search_sqdis[0] < DISTANCE_SQ_THRESHOLD)
            {
                closest_point_ind = point_search_ind[0];
                int closest_point_scan_id = int(corner_points_last->points[closest_point_ind].intensity);

                double min_point_sqdis2 = DISTANCE_SQ_THRESHOLD;
                // search in the direction of increasing scan line
                for (int j = closest_point_ind + 1; j < (int)corner_points_last->points.size(); ++j)
                {
                    // if in the same scan line, continue
                    if (int(corner_points_last->points[j].intensity) <= closest_point_scan_id)
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(corner_points_last->points[j].intensity) > (closest_point_scan_id + NEARBY_SCAN))
                        break;

                    double point_sqdis = sqrSum(corner_points_last->points[j].x - point_sel.x,
                                                corner_points_last->points[j].y - point_sel.y,
                                                corner_points_last->points[j].z - point_sel.z);

                    if (point_sqdis < min_point_sqdis2)
                    {
                        // find nearer point
                        min_point_sqdis2 = point_sqdis;
                        min_point_ind2 = j;
                    }
                }

                // search in the direction of decreasing scan line
                for (int j = closest_point_ind - 1; j >= 0; --j)
                {
                    // if in the same scan line, continue
                    if (int(corner_points_last->points[j].intensity) >= closest_point_scan_id)
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(corner_points_last->points[j].intensity) < (closest_point_scan_id - NEARBY_SCAN))
                        break;

                    double point_sqdis = sqrSum(corner_points_last->points[j].x - point_sel.x,
                                                corner_points_last->points[j].y - point_sel.y,
                                                corner_points_last->points[j].z - point_sel.z);

                    if (point_sqdis < min_point_sqdis2)
                    {
                        // find nearer point
                        min_point_sqdis2 = point_sqdis;
                        min_point_ind2 = j;
                    }
                }
            }
            if (min_point_ind2 >= 0) // both closest_point_ind and min_point_ind2 is valid
            {
                Eigen::Vector3d curr_point(corner_points_sharp->points[i].x,
                                           corner_points_sharp->points[i].y,
                                           corner_points_sharp->points[i].z);
                Eigen::Vector3d last_point_a(corner_points_last->points[closest_point_ind].x,
                                             corner_points_last->points[closest_point_ind].y,
                                             corner_points_last->points[closest_point_ind].z);
                Eigen::Vector3d last_point_b(corner_points_last->points[min_point_ind2].x,
                                             corner_points_last->points[min_point_ind2].y,
                                             corner_points_last->points[min_point_ind2].z);

                double s;
                if (DISTORTION)
                    s = (corner_points_sharp->points[i].intensity - int(corner_points_sharp->points[i].intensity)) / SCAN_PERIOD;
                else
                    s = 1.0;
                ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                num_corner_correspondence++;
            }
        }

        // find correspondence for plane features
        for (int i = 0; i < surf_points_sharp_num; ++i)
        {
            TransformToStart(&(surf_points_flat->points[i]), &point_sel, Pose(q_prev_cur, t_prev_cur));
            kdtree_surf_last->nearestKSearch(point_sel, 1, point_search_ind, point_search_sqdis);

            int closest_point_ind = -1, min_point_ind2 = -1, minPointInd3 = -1;
            if (point_search_sqdis[0] < DISTANCE_SQ_THRESHOLD)
            {
                closest_point_ind = point_search_ind[0];

                // get closest point's scan ID
                int closest_point_scan_id = int(surf_points_last->points[closest_point_ind].intensity);
                double min_point_sqdis2 = DISTANCE_SQ_THRESHOLD, min_point_sqdis3 = DISTANCE_SQ_THRESHOLD;

                // search in the direction of increasing scan line
                for (int j = closest_point_ind + 1; j < (int)surf_points_last->points.size(); ++j)
                {
                    // if not in nearby scans, end the loop
                    if (int(surf_points_last->points[j].intensity) > (closest_point_scan_id + NEARBY_SCAN))
                        break;

                    double point_sqdis = sqrSum(surf_points_last->points[j].x - point_sel.x,
                                                surf_points_last->points[j].y - point_sel.y,
                                                surf_points_last->points[j].z - point_sel.z);
                    // if in the same or lower scan line
                    if (int(surf_points_last->points[j].intensity) <= closest_point_scan_id && point_sqdis < min_point_sqdis2)
                    {
                        min_point_sqdis2 = point_sqdis;
                        min_point_ind2 = j;
                    }
                    // if in the higher scan line
                    else if (int(surf_points_last->points[j].intensity) > closest_point_scan_id && point_sqdis < min_point_sqdis3)
                    {
                        min_point_sqdis3 = point_sqdis;
                        minPointInd3 = j;
                    }
                }

                // search in the direction of decreasing scan line
                for (int j = closest_point_ind - 1; j >= 0; --j)
                {
                    // if not in nearby scans, end the loop
                    if (int(surf_points_last->points[j].intensity) < (closest_point_scan_id - NEARBY_SCAN))
                        break;

                    double point_sqdis = sqrSum(surf_points_last->points[j].x - point_sel.x,
                                                surf_points_last->points[j].y - point_sel.y,
                                                surf_points_last->points[j].z - point_sel.z);
                    // if in the same or higher scan line
                    if (int(surf_points_last->points[j].intensity) >= closest_point_scan_id && point_sqdis < min_point_sqdis2)
                    {
                        min_point_sqdis2 = point_sqdis;
                        min_point_ind2 = j;
                    }
                    else if (int(surf_points_last->points[j].intensity) < closest_point_scan_id && point_sqdis < min_point_sqdis3)
                    {
                        // find nearer point
                        min_point_sqdis3 = point_sqdis;
                        minPointInd3 = j;
                    }
                }

                if (min_point_ind2 >= 0 && minPointInd3 >= 0)
                {

                    Eigen::Vector3d curr_point(surf_points_flat->points[i].x,
                                               surf_points_flat->points[i].y,
                                               surf_points_flat->points[i].z);
                    Eigen::Vector3d last_point_a(surf_points_last->points[closest_point_ind].x,
                                                 surf_points_last->points[closest_point_ind].y,
                                                 surf_points_last->points[closest_point_ind].z);
                    Eigen::Vector3d last_point_b(surf_points_last->points[min_point_ind2].x,
                                                 surf_points_last->points[min_point_ind2].y,
                                                 surf_points_last->points[min_point_ind2].z);
                    Eigen::Vector3d last_point_c(surf_points_last->points[minPointInd3].x,
                                                 surf_points_last->points[minPointInd3].y,
                                                 surf_points_last->points[minPointInd3].z);
                    double s;
                    if (DISTORTION)
                        s = (surf_points_flat->points[i].intensity - int(surf_points_flat->points[i].intensity)) / SCAN_PERIOD;
                    else
                        s = 1.0;
                    ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                    num_plane_correspondence++;
                }
            }
        }

        printf("iter: %d, coner_corre: %d, plane_corre: %d \n", opti_counter, num_corner_correspondence, num_plane_correspondence);
        // printf("data association time %f ms \n", t_data.toc());
        if ((num_corner_correspondence + num_plane_correspondence) < 10)
        {
            printf("less correspondence! *************************************************\n");
        }

        // step 3: optimization
        TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // printf("solver time %f ms \n", t_solver.toc());
    }
    // printf("optimization twice time %f \n", t_opt.toc());

    Pose pose_prev_cur(q_prev_cur, t_prev_cur);
    // std::cout << "tracker transform: " << pose_prev_cur << std::endl;
    printf("whole tracker time %f ms \n", t_whole.toc());

    return pose_prev_cur;
}



//
