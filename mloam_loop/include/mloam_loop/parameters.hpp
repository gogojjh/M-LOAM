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

#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <map>
#include <cassert>
#include <cstdio>

using namespace std;

extern int RESULT_SAVE;
extern std::string OUTPUT_FOLDER;
extern std::string MLOAM_LOOP_PATH;

extern int LOOP_SKIP_INTERVAL;
extern int LOOP_HISTORY_SEARCH_NUM;
extern double LOOP_DISTANCE_THRESHOLD;
extern double LOOP_OPTI_COST_THRESHOLD;
extern double LOOP_TEMPORAL_CONSISTENCY_THRESHOLD;
extern double LOOP_GLOBAL_REGISTRATION_THRESHOLD;
extern double LOOP_LOCAL_REGISTRATION_THRESHOLD;

extern int VISUALIZE_IMAGE;
extern int LOAD_PREVIOUS_POSE_GRAPH;
extern int LOOP_SAVE_PCD;
extern string POSE_GRAPH_SAVE_PATH;

extern int VISUALIZATION_SHIFT_X;
extern int VISUALIZATION_SHIFT_Y;

// scan context
extern double LIDAR_HEIGHT;
extern int PC_NUM_RING;
extern int PC_NUM_SECTOR;
extern double PC_MAX_RADIUS;
extern double PC_UNIT_SECTORANGLE;
extern double PC_UNIT_RINGGAP;
extern int NUM_EXCLUDE_RECENT;
extern int NUM_CANDIDATES_FROM_TREE;
extern double SEARCH_RATIO;
extern double SC_DIST_THRES;
extern int TREE_MAKING_PERIOD;

// registration
extern double NORMAL_RADIUS;
extern double FPFH_RADIUS;
extern double DIV_FACTOR;
extern double USE_ABSOLUTE_SCALE;
extern double MAX_CORR_DIST;
extern double ITERATION_NUMBER;
extern double TUPLE_SCALE;
extern double TUPLE_MAX_CNT;

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

