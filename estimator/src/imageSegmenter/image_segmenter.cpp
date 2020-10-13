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

#include "image_segmenter.hpp"

using namespace common;

void ImageSegmenter::setParameter(const int &vertical_scans,
                                  const int &horizon_scans,
                                  const int &min_cluster_size,
                                  const int &segment_valid_point_num,
                                  const int &segment_valid_line_num)
{
    vertical_scans_ = vertical_scans;
    horizon_scans_ = horizon_scans;
    min_cluster_size_ = min_cluster_size;
    if (vertical_scans_ == 16)
    {
        ang_res_x_ = 360.0 / horizon_scans_;
        ang_res_y_ = 2.0;
        ang_bottom_ = 15.0 + 0.1;
        ground_scan_id_ = 7;
        
        segment_alphax_ = ang_res_x_ / 180.0 * M_PI;
        segment_alphay_ = ang_res_y_ / 180.0 * M_PI;
        segment_valid_point_num_ = segment_valid_point_num;
        segment_valid_line_num_ = segment_valid_line_num;
    }
    else if (vertical_scans_ == 32)
    {
        ang_res_x_ = 360.0 / horizon_scans_;
        ang_res_y_ = 41.33 / float(vertical_scans_ - 1);
        ang_bottom_ = 30.0 + 0.67;
        ground_scan_id_ = 20;
        
        segment_alphax_ = ang_res_x_ / 180.0 * M_PI;
        segment_alphay_ = ang_res_y_ / 180.0 * M_PI;
        segment_valid_point_num_ = segment_valid_point_num;
        segment_valid_line_num_ = segment_valid_line_num;
    }
    else if (vertical_scans_ == 64)
    {
        ang_res_x_ = 360.0 / horizon_scans_;
        ang_res_y_ = FLT_MAX;
        ground_scan_id_ = 63; // 7-64 is ground
        
        segment_alphax_ = ang_res_x_ / 180.0 * M_PI;
        segment_valid_point_num_ = segment_valid_point_num;
        segment_valid_line_num_ = segment_valid_line_num;
    }
    printf("[ImageSegmenter param] v_scans:%d, h_scans:%d, c_size:%d\n", 
        vertical_scans, horizon_scans_, min_cluster_size_);
}

// VLP-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 2.0;
// extern const float ang_bottom = 15.0+0.1;
// extern const int groundScanInd = 7;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0 / float(Horizon_SCAN);
// extern const float ang_res_y = 41.33 / float(N_SCAN - 1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet, please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;
