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
#include "mloam_loop/parameters.h"

int RESULT_SAVE;
std::string OUTPUT_FOLDER;

int LOOP_KEYFRAME_INTERVAL;
int LOOP_HISTORY_SEARCH_NUM;
double LOOP_OPTI_COST_THRESHOLD;

// scan context
double LIDAR_HEIGHT;
int PC_NUM_RING;
int PC_NUM_SECTOR;
double PC_MAX_RADIUS;
double PC_UNIT_SECTORANGLE;
double PC_UNIT_RINGGAP;
int NUM_EXCLUDE_RECENT;
int NUM_CANDIDATES_FROM_TREE;
double SEARCH_RATIO;
double SC_DIST_THRES;
int TREE_MAKING_PERIOD;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL)
    {
        std::cout << common::YELLOW << "[mloam_loop] config_file dosen't exist; wrong config_file path" << common::RESET << std::endl;
        ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    LOOP_KEYFRAME_INTERVAL = fsSettings["loop_keyframe_interval"];
    LOOP_HISTORY_SEARCH_NUM = fsSettings["loop_history_search_num"];
    LOOP_OPTI_COST_THRESHOLD = fsSettings["loop_opti_cost_threshold"];

    // scan context
    LIDAR_HEIGHT = fsSettings["lidar_height"];

    PC_NUM_RING = fsSettings["pc_num_ring"];
    PC_NUM_SECTOR = fsSettings["pc_num_sector"];
    PC_MAX_RADIUS = fsSettings["pc_max_radius"];
    PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);    

    NUM_EXCLUDE_RECENT = fsSettings["num_exclude_recent"];
    NUM_CANDIDATES_FROM_TREE = fsSettings["num_candidates_from_tree"];

    SEARCH_RATIO = fsSettings["search_ratio"];
    SC_DIST_THRES = fsSettings["sc_dist_thres"];
    TREE_MAKING_PERIOD = fsSettings["tree_making_period"];

    fsSettings.release();
}
