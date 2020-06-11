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

#include "image_segmenter.h"

using namespace common;

void ImageSegmenter::segmentCloud(const PointCloud &laser_cloud_in, PointCloud &laser_cloud_out) const
{
    // set specific parameters
    PointCloud::Ptr cloud_matrix(new PointCloud);
    cloud_matrix->points.resize(vertical_scans_ * horizon_scans_);

    cv::Mat range_mat = cv::Mat(vertical_scans_, horizon_scans_, CV_32F, cv::Scalar::all(FLT_MAX));
    cv::Mat label_mat = cv::Mat(vertical_scans_, horizon_scans_, CV_32S, cv::Scalar::all(0));
    int label_count = 1;

    uint16_t *all_pushed_indx = new uint16_t[vertical_scans_ * horizon_scans_];;
    uint16_t *all_pushed_indy = new uint16_t[vertical_scans_ * horizon_scans_];;

    uint16_t *queue_indx = new uint16_t[vertical_scans_ * horizon_scans_];
    uint16_t *queue_indy = new uint16_t[vertical_scans_ * horizon_scans_];

    int *queue_indx_last_negi = new int[vertical_scans_ * horizon_scans_];
    int *queue_indy_last_negi = new int[vertical_scans_ * horizon_scans_];
    float *queue_last_dis = new float[vertical_scans_ * horizon_scans_];

    // convert point cloud to a range image
    float vertical_angle, horizon_angle, range;
    size_t row_id, column_id;
    size_t cloud_size = 0;
    for (size_t i = 0; i < laser_cloud_in.points.size(); i++)
    {
        const auto &point = laser_cloud_in.points[i];
        vertical_angle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        horizon_angle = atan2(point.x, point.y) * 180 / M_PI;

        row_id = static_cast<size_t>((vertical_angle + ang_bottom_) / ang_res_y_);
        column_id = -round((horizon_angle - 90.0) / ang_res_x_) + horizon_scans_ / 2;
        if (row_id < 0 || row_id >= vertical_scans_)
            continue;
        if (column_id >= horizon_scans_)
            column_id -= horizon_scans_;
        if (column_id < 0 || column_id >= horizon_scans_)
            continue;

        range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        range_mat.at<float>(row_id, column_id) = range;
        int index = column_id + row_id * horizon_scans_;
        cloud_matrix->points[index] = point;
        cloud_size++;
    }
    for (size_t i = 0; i < vertical_scans_; i++)
        for (size_t j = 0; j < horizon_scans_; j++)
            if (range_mat.at<float>(i, j) == FLT_MAX)
                label_mat.at<int>(i, j) = -1;

    // label ground points
    for (size_t i = 0; i < ground_scan_id_; i++)
        for (size_t j = 0; j < horizon_scans_; j++)
            if (label_mat.at<int>(i, j) == 0)
                label_mat.at<int>(i, j) = label_count;
    label_count++;

    // BFS to search nearest neighbors
    for (size_t i = 0; i < vertical_scans_; i++)
    {
        for (size_t j = 0; j < horizon_scans_; j++)
        {
            if (label_mat.at<int>(i,j) == 0) 
            {
                int row = i;
                int col = j;

                float d1, d2, alpha, angle, dist;
                int from_indx, from_indy, this_indx, this_indy;
                bool line_count_flag[vertical_scans_] = {false};

                queue_indx[0] = row;
                queue_indy[0] = col;
                queue_indx_last_negi[0] = 0;
                queue_indy_last_negi[0] = 0;
                queue_last_dis[0] = 0;
                int queue_size = 1;
                int queue_start_ind = 0;
                int queue_end_ind = 1;

                all_pushed_indx[0] = row;
                all_pushed_indy[0] = col;
                int all_pushed_ind_size = 1;

                // find the neighbor connecting clusters in range image, bfs
                while (queue_size > 0)
                {
                    from_indx = queue_indx[queue_start_ind];
                    from_indy = queue_indy[queue_start_ind];
                    --queue_size;
                    ++queue_start_ind;
                    label_mat.at<int>(from_indx, from_indy) = label_count;
                    for (auto iter = neighbor_iterator_.begin(); iter != neighbor_iterator_.end(); ++iter)
                    {
                        this_indx = from_indx + iter->first;
                        this_indy = from_indy + iter->second;
                        if (this_indx < 0 || this_indx >= vertical_scans_)
                            continue;
                        if (this_indy < 0)
                            this_indy = horizon_scans_ - 1;
                        if (this_indy >= horizon_scans_)
                            this_indy = 0;
                        if (label_mat.at<int>(this_indx, this_indy) != 0)
                            continue;

                        d1 = std::max(range_mat.at<float>(from_indx, from_indy),
                                      range_mat.at<float>(this_indx, this_indy));
                        d2 = std::min(range_mat.at<float>(from_indx, from_indy),
                                      range_mat.at<float>(this_indx, this_indy));
                        dist = sqrt(d1 * d1 + d2 * d2 - 2 * d1 * d2 * cos(alpha));
                        alpha = iter->first == 0 ? segment_alphax_ : segment_alphay_;
                        angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));
                        if (angle > SEGMENT_THETA)
                        {
                            queue_indx[queue_end_ind] = this_indx;
                            queue_indy[queue_end_ind] = this_indy;
                            queue_indx_last_negi[queue_end_ind] = iter->first;
                            queue_indy_last_negi[queue_end_ind] = iter->second;
                            queue_last_dis[queue_end_ind] = dist;
                            queue_size++;
                            queue_end_ind++;

                            label_mat.at<int>(this_indx, this_indy) = label_count;
                            line_count_flag[this_indx] = true;

                            all_pushed_indx[all_pushed_ind_size] = this_indx;
                            all_pushed_indy[all_pushed_ind_size] = this_indy;
                            all_pushed_ind_size++;
                        }
                        else if ((iter->second == 0) && (queue_indy_last_negi[queue_start_ind] == 0)) // at the same beam
                        {
                            float dist_last = queue_last_dis[queue_start_ind];
                            if ((dist_last / dist <= 1.2) && ((dist_last / dist >= 0.8))) // inside a plane
                            {
                                queue_indx[queue_end_ind] = this_indx;
                                queue_indy[queue_end_ind] = this_indy;
                                queue_indx_last_negi[queue_end_ind] = iter->first;
                                queue_indy_last_negi[queue_end_ind] = iter->second;
                                queue_last_dis[queue_end_ind] = dist;
                                queue_size++;
                                queue_end_ind++;

                                label_mat.at<int>(this_indx, this_indy) = label_count;
                                line_count_flag[this_indx] = true;

                                all_pushed_indx[all_pushed_ind_size] = this_indx;
                                all_pushed_indy[all_pushed_ind_size] = this_indy;
                                all_pushed_ind_size++;
                            }
                        }
                    }
                }
                
                bool feasible_segment = false;
                if (all_pushed_ind_size >= min_cluster_size_) // cluster_size > min_cluster_size_
                {
                    feasible_segment = true;
                }
                else if (all_pushed_ind_size >= segment_valid_point_num_) // line_size > line_mini_size
                {
                    int line_count = 0;
                    for (size_t i = 0; i < vertical_scans_; i++)
                        if (line_count_flag[i]) line_count++;

                    if (line_count >= segment_valid_line_num_) feasible_segment = true;
                }

                if (feasible_segment)
                {
                    label_count++;
                }
                else
                {
                    for (size_t i = 0; i < all_pushed_ind_size; ++i)
                    {
                        label_mat.at<int>(all_pushed_indx[i], all_pushed_indy[i]) = 999999;
                    }
                }
            }
        }
    }

    // convert segmented points to point cloud
    laser_cloud_out.clear();
    for (size_t i = 0; i < vertical_scans_; i++)
        for (size_t j = 0; j < horizon_scans_; j++)
            if ((label_mat.at<int>(i, j) > 0) && (label_mat.at<int>(i, j) != 999999))
                laser_cloud_out.push_back(cloud_matrix->points[j + i*horizon_scans_]);
    // printf("input cloud size:%d, output cloud size:%d\n", laser_cloud_in.size(), laser_cloud_out.size());
}



