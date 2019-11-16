/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string OUTPUT_FOLDER;
std::string MLOAM_ODOM_PATH;
std::string MLOAM_MAP_PATH;
std::string MLOAM_GT_PATH;
std::string EX_CALIB_RESULT_PATH;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

// LiDAR
int NUM_OF_LASER;
int N_SCANS;

int IDX_REF;

int WINDOW_SIZE;
int OPT_WINDOW_SIZE;

float SCAN_PERIOD;
float DISTANCE_SQ_THRESHOLD;
float NEARBY_SCAN;

std::string CLOUD0_TOPIC, CLOUD1_TOPIC;
float LASER_SYNC_THRESHOLD;
double ROI_RANGE;

std::vector<Eigen::Matrix3d> RBL;
std::vector<Eigen::Quaterniond> QBL;
std::vector<Eigen::Vector3d> TBL;
std::vector<double> TDBL;

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

int OPTIMAL_EXTRINSIC;

int EVALUATE_RESIDUAL;

int PCL_VIEWER;
int PCL_VIEWER_NORMAL_RATIO;

int OPTIMAL_ODOMETRY;
int N_CUMU_FEATURE;

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
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["cloud0_topic"] >> CLOUD0_TOPIC;
    fsSettings["cloud1_topic"] >> CLOUD1_TOPIC;

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    MLOAM_ODOM_PATH = OUTPUT_FOLDER + fsSettings["mloam_odom_path"];
	MLOAM_MAP_PATH = OUTPUT_FOLDER + fsSettings["mloam_map_path"];
    MLOAM_GT_PATH = OUTPUT_FOLDER + fsSettings["mloam_gt_path"];
    std::cout << "result path " << MLOAM_ODOM_PATH << ", " << MLOAM_MAP_PATH << std::endl;

    NUM_OF_LASER = fsSettings["num_of_laser"];
    printf("Laser number %d\n", NUM_OF_LASER);
    if(NUM_OF_LASER != 1 && NUM_OF_LASER != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }

    WINDOW_SIZE = fsSettings["window_size"];
    OPT_WINDOW_SIZE = fsSettings["opt_window_size"];
    printf("window_size: %d, opt_window_size: %d\n", WINDOW_SIZE, OPT_WINDOW_SIZE);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    OPTIMAL_EXTRINSIC = fsSettings["optimal_extrinsic"];
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "extrinsic_parameter.csv";
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("Have no prior about extrinsic param, calibrate extrinsic param");
        for (int i = 0; i < NUM_OF_LASER; i++)
        {
            QBL.push_back(Eigen::Quaterniond::Identity());
            TBL.push_back(Eigen::Vector3d::Zero());
        }
    }
    else
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN("Please optimize extrinsic param around initial guess!");
        }
        if (ESTIMATE_EXTRINSIC == 0)
        {
            ROS_WARN("Fix extrinsic param ");
        }

        cv::Mat cv_T;
        fsSettings["body_T_laser"] >> cv_T;
        for (int i = 0; i < NUM_OF_LASER; i++)
        {
            QBL.push_back(Eigen::Quaterniond(cv_T.ptr<double>(i)[3], cv_T.ptr<double>(i)[0], cv_T.ptr<double>(i)[1], cv_T.ptr<double>(i)[2]));
            TBL.push_back(Eigen::Vector3d(cv_T.ptr<double>(i)[4], cv_T.ptr<double>(i)[5], cv_T.ptr<double>(i)[6]));
        }
    }
    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);

    cv::Mat cv_TD;
    fsSettings["td"] >> cv_TD;
    for (int i = 0; i < NUM_OF_LASER; i++) TDBL.push_back(cv_TD.ptr<double>(0)[i]);
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset");
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset");

    LASER_SYNC_THRESHOLD = fsSettings["laser_sync_threshold"];
    N_SCANS = fsSettings["n_scans"];
    ROI_RANGE = fsSettings["roi_range"];

    IDX_REF = fsSettings["idx_ref"];

    SCAN_PERIOD = fsSettings["scan_period"];
    DISTANCE_SQ_THRESHOLD = fsSettings["distance_sq_threshold"];
    NEARBY_SCAN = fsSettings["nearby_scan"];

    PLANAR_MOVEMENT = fsSettings["planar_movement"];

    MIN_MATCH_SQ_DIS = fsSettings["min_match_sq_dis"];
    MIN_PLANE_DIS = fsSettings["min_plane_dis"];

    MARGINALIZATION_FACTOR = fsSettings["marginalization_factor"];
    POINT_PLANE_FACTOR = fsSettings["point_plane_factor"];
    POINT_EDGE_FACTOR = fsSettings["point_edge_factor"];
    PRIOR_FACTOR = fsSettings["prior_factor"];
    PRIOR_FACTOR_POS = fsSettings["prior_factor_pos"];
    PRIOR_FACTOR_ROT = fsSettings["prior_factor_rot"];
    CHECK_JACOBIAN = fsSettings["check_jacobian"];

    EVALUATE_RESIDUAL = fsSettings["evaluate_residual"];

    PCL_VIEWER = fsSettings["pcl_viewer"];
    PCL_VIEWER_NORMAL_RATIO = fsSettings["pcl_viewer_normal_ratio"];

    OPTIMAL_ODOMETRY = fsSettings["optimal_odometry"];
    N_CUMU_FEATURE = fsSettings["n_cumu_feature"];

    fsSettings.release();
}
