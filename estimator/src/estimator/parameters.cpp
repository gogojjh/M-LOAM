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

#include "parameters.h"

int MLOAM_RESULT_SAVE;
std::string OUTPUT_FOLDER;
std::string MLOAM_ODOM_PATH;
std::string MLOAM_MAP_PATH;
std::string MLOAM_GPS_PATH;
std::string MLOAM_GT_PATH;
std::string EX_CALIB_RESULT_PATH;
std::string EX_CALIB_EIG_PATH;

int MULTIPLE_THREAD;

double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;

// segmentation
int SEGMENT_CLOUD;
int HORIZON_SCAN;
int MIN_CLUSTER_SIZE;
int MIN_LINE_SIZE;
int SEGMENT_VALID_POINT_NUM;
int SEGMENT_VALID_LINE_NUM;
float SEGMENT_THETA;

// LiDAR
size_t NUM_OF_LASER;
size_t N_SCANS;
size_t IDX_REF;

int WINDOW_SIZE;
int OPT_WINDOW_SIZE;

int DISTORTION;
float SCAN_PERIOD;
float DISTANCE_SQ_THRESHOLD;
float NEARBY_SCAN;

std::string CLOUD0_TOPIC, CLOUD1_TOPIC;
std::vector<std::string> CLOUD_TOPIC;

float LASER_SYNC_THRESHOLD;
double ROI_RANGE;

std::vector<Eigen::Quaterniond> QBL;
std::vector<Eigen::Vector3d> TBL;
std::vector<double> TDBL;

// odometry
int PLANAR_MOVEMENT;

float MIN_MATCH_SQ_DIS;
float MIN_PLANE_DIS;

int MARGINALIZATION_FACTOR;
int POINT_PLANE_FACTOR;
int POINT_EDGE_FACTOR;
int PRIOR_FACTOR;
double PRIOR_FACTOR_POS;
double PRIOR_FACTOR_ROT;
int CHECK_JACOBIAN;

int PCL_VIEWER;
int PCL_VIEWER_NORMAL_RATIO;

int N_CUMU_FEATURE;
double LAMBDA_INITIAL;
double LAMBDA_THRE_CALIB;
int N_CALIB;
float ODOM_GF_RATIO;

int SKIP_NUM_ODOM_PUB;
int LM_OPT_ENABLE;

// mapping
float MAP_CORNER_RES;
float MAP_SURF_RES;
float MAP_OUTLIER_RES;
float MAP_SUR_KF_RES;
float MAP_EIG_THRE;
float MAP_DEG_THRE;

float DISTANCE_KEYFRAMES;
float ORIENTATION_KEYFRAMES;
float SURROUNDING_KF_RADIUS;

float UCT_EXT_RATIO;
std::vector<Eigen::Matrix<double, 6, 6> > COV_EXT;
Eigen::Matrix<double, 3, 3> COV_MEASUREMENT;
double TRACE_THRESHOLD_MAPPING;

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
        std::cout << common::YELLOW << "config_file dosen't exist; wrong config_file path" << common::RESET << std::endl;
        ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    int num_of_laser = fsSettings["num_of_laser"];
    assert(num_of_laser >= 0);
    NUM_OF_LASER = (size_t)num_of_laser;
    printf("laser number %lu\n", NUM_OF_LASER);

    cv::FileNode node_cloud_topic = fsSettings["cloud_topic"];
    CLOUD_TOPIC.resize(NUM_OF_LASER);
    for (size_t i = 0; i < NUM_OF_LASER; i++)
    {
        CLOUD_TOPIC[i] = (std::string)node_cloud_topic[i];
        printf("cloud_topic: %s\n", CLOUD_TOPIC[i].c_str());
    }

    WINDOW_SIZE = fsSettings["window_size"];
    OPT_WINDOW_SIZE = fsSettings["opt_window_size"];
    printf("window_size: %d, opt_window_size: %d\n", WINDOW_SIZE, OPT_WINDOW_SIZE);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    QBL.resize(NUM_OF_LASER);
    TBL.resize(NUM_OF_LASER);
    if (ESTIMATE_EXTRINSIC == 2)
    {
        std::cout << common::YELLOW << "have no prior about extrinsic param, calibrate extrinsic param" << common::RESET << std::endl;
        for (int i = 0; i < NUM_OF_LASER; i++)
        {
            QBL[i] = Eigen::Quaterniond::Identity();
            TBL[i] = Eigen::Vector3d::Zero();
        }
    } else
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            std::cout << common::YELLOW << "please optimize extrinsic param around initial guess!" << common::RESET << std::endl;
        }
        if (ESTIMATE_EXTRINSIC == 0)
        {
            std::cout << common::YELLOW << "fix extrinsic param " << common::RESET << std::endl;
        }

        cv::Mat cv_T;
        fsSettings["body_T_laser"] >> cv_T;
        for (int i = 0; i < NUM_OF_LASER; i++)
        {
            QBL[i] = Eigen::Quaterniond(cv_T.ptr<double>(i)[3], cv_T.ptr<double>(i)[0], cv_T.ptr<double>(i)[1], cv_T.ptr<double>(i)[2]);
            TBL[i] = Eigen::Vector3d(cv_T.ptr<double>(i)[4], cv_T.ptr<double>(i)[5], cv_T.ptr<double>(i)[6]);
        }
    }

    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        std::cout << "Unsynchronized sensors, online estimate time offset" << std::endl;
    else
        std::cout << "Synchronized sensors, fix time offset" << std::endl;

    cv::Mat cv_TD;
    fsSettings["td"] >> cv_TD;
    TDBL.resize(NUM_OF_LASER);
    for (size_t i = 0; i < NUM_OF_LASER; i++) TDBL[i] = cv_TD.ptr<double>(0)[i]; 

    LASER_SYNC_THRESHOLD = fsSettings["laser_sync_threshold"];

    int n_scans = fsSettings["n_scans"];
    assert(n_scans >= 0);
    N_SCANS = (size_t)n_scans;

    // odometry
    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];

    ROI_RANGE = fsSettings["roi_range"];

    SEGMENT_CLOUD = fsSettings["segment_cloud"];
    HORIZON_SCAN = fsSettings["horizon_scan"];
    MIN_CLUSTER_SIZE = fsSettings["min_cluster_size"];
    MIN_LINE_SIZE = fsSettings["min_line_size"];
    SEGMENT_VALID_POINT_NUM = fsSettings["segment_valid_point_num"];
    SEGMENT_VALID_LINE_NUM = fsSettings["segment_valid_line_num"];
    SEGMENT_THETA = fsSettings["segment_theta"];

    int idx_ref = fsSettings["idx_ref"];
    assert(idx_ref >= 0);
    IDX_REF = (size_t)idx_ref;

    SCAN_PERIOD = fsSettings["scan_period"];
    DISTANCE_SQ_THRESHOLD = fsSettings["distance_sq_threshold"];
    NEARBY_SCAN = fsSettings["nearby_scan"];
    DISTORTION = fsSettings["distortion"];

    PLANAR_MOVEMENT = fsSettings["planar_movement"];

    MIN_MATCH_SQ_DIS = fsSettings["min_match_sq_dis"];
    MIN_PLANE_DIS = fsSettings["min_plane_dis"];

    MARGINALIZATION_FACTOR = fsSettings["marginalization_factor"];
    POINT_PLANE_FACTOR = fsSettings["point_plane_factor"];
    POINT_EDGE_FACTOR = fsSettings["point_edge_factor"];
    PRIOR_FACTOR = fsSettings["prior_factor"];
    PRIOR_FACTOR_POS = fsSettings["prior_factor_pos"];
    PRIOR_FACTOR_ROT = fsSettings["prior_factor_rot"];
    CHECK_JACOBIAN = 0;

    PCL_VIEWER = fsSettings["pcl_viewer"];
    PCL_VIEWER_NORMAL_RATIO = fsSettings["pcl_viewer_normal_ratio"];

    N_CUMU_FEATURE = fsSettings["n_cumu_feature"];
    LAMBDA_INITIAL = fsSettings["lambda_initial"];
    LAMBDA_THRE_CALIB = fsSettings["lambda_thre_calib"];
    N_CALIB = fsSettings["n_calib"];
    ODOM_GF_RATIO = fsSettings["odom_gf_ratio"];
    if (ODOM_GF_RATIO == 0) ODOM_GF_RATIO = 1.0;

    SKIP_NUM_ODOM_PUB = fsSettings["skip_num_odom_pub"];
    if (SKIP_NUM_ODOM_PUB == 0) SKIP_NUM_ODOM_PUB = 1;

    LM_OPT_ENABLE = fsSettings["lm_opt_enable"];

    // mapping 
    MAP_CORNER_RES = fsSettings["map_corner_res"];
    MAP_SURF_RES = fsSettings["map_surf_res"];
    MAP_OUTLIER_RES = fsSettings["map_outlier_res"];
    MAP_SUR_KF_RES = fsSettings["map_sur_kf_res"];
    MAP_EIG_THRE = fsSettings["map_eig_thre"];
    MAP_DEG_THRE = fsSettings["map_deg_thre"];
    printf("map corner resolution:%f, surf resolution:%f, surround kf resolution:%f\n",
            MAP_CORNER_RES, MAP_SURF_RES, MAP_SUR_KF_RES);

    DISTANCE_KEYFRAMES = fsSettings["distance_keyframes"];
    ORIENTATION_KEYFRAMES = fsSettings["orientation_keyframes"];
    SURROUNDING_KF_RADIUS = fsSettings["surrounding_kf_radius"];
    printf("map kf radius: %f, kf dis:%f, ori:%f\n", SURROUNDING_KF_RADIUS, DISTANCE_KEYFRAMES, ORIENTATION_KEYFRAMES);

    UCT_EXT_RATIO = fsSettings["uct_ext_ratio"];
    printf("uct ext ratio: %f\n", UCT_EXT_RATIO);
    cv::Mat cv_uct;
    COV_EXT.resize(NUM_OF_LASER);
    fsSettings["uct_ext"] >> cv_uct;
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        Eigen::Matrix<double, 6, 1> vec_uct_ext; // rotation, translation, point
        vec_uct_ext << cv_uct.ptr<double>(i)[0], cv_uct.ptr<double>(i)[1], cv_uct.ptr<double>(i)[2],
            cv_uct.ptr<double>(i)[3], cv_uct.ptr<double>(i)[4], cv_uct.ptr<double>(i)[5];
        COV_EXT[i] = vec_uct_ext.asDiagonal() * UCT_EXT_RATIO;
    }

    fsSettings["uct_measurement"] >> cv_uct;
    Eigen::Vector3d vec_uct_measurement;
    vec_uct_measurement << cv_uct.ptr<double>(0)[0], cv_uct.ptr<double>(0)[1], cv_uct.ptr<double>(0)[2];
    COV_MEASUREMENT = vec_uct_measurement.asDiagonal();

    TRACE_THRESHOLD_MAPPING = fsSettings["trace_threshold_mapping"];

    fsSettings.release();
}
