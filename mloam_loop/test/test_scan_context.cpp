#include <iostream>
#include <string>
#include <memory>
#include <mutex>
#include <queue>
#include <stack>
#include <thread>
#include <signal.h>
#include <vector>
#include <algorithm>
#include <iterator>
#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include "mloam_loop/utility/tic_toc.h"
#include "mloam_loop/scan_context/scan_context.hpp"
#include "mloam_loop/parameters.hpp"

using namespace std;

int LOOP_KEYFRAME_INTERVAL;
int LOOP_HISTORY_SEARCH_NUM;
double LOOP_DISTANCE_THRESHOLD;
double LOOP_OPTI_COST_THRESHOLD;
double LOOP_TEMPORAL_CONSISTENCY_THRESHOLD;
double LOOP_GEOMETRIC_CONSISTENCY_THRESHOLD;

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

SCManager sc_manager;

std::vector<std::string> ref_filename;
std::vector<std::string> target_filename;

void ground_filter(const pcl::PointCloud<pcl::PointXYZI> &pc_in, pcl::PointCloud<pcl::PointXYZI> &pc_out)
{
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(pc_in));
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-3.0, 50.0);
    pass.filter(pc_out);
}

std::pair<bool, int> checkTemporalConsistency()
{
    size_t que_idx = sc_manager.getDataBaseSize() - 1;
    // std::cout << "que_idx: " << que_idx << std::endl;

    QueryResult qr = sc_manager.detectLoopClosureID(que_idx); // detect_result.second: yaw
    int match_idx = qr.match_index_;
    if (match_idx != -1)
    {
        bool tc_flag = true;
        std::vector<int> all_match_idx;
        all_match_idx.push_back(match_idx);
        for (size_t reque_idx = que_idx - 5; reque_idx < que_idx; reque_idx++)
        {
            // std::cout << "reque_idx: " << reque_idx << std::endl;
            QueryResult qr = sc_manager.detectLoopClosureID(reque_idx);
            int rematch_idx = qr.match_index_;
            // if (rematch_idx != -1) std::cout << ref_filename[rematch_idx] << std::endl;
            if ((rematch_idx == -1) || (abs(match_idx - rematch_idx) > LOOP_TEMPORAL_CONSISTENCY_THRESHOLD))
            {
                tc_flag = false;
                break;
            }
            all_match_idx.push_back(rematch_idx);
        }
        if (tc_flag)
        {
            for (size_t i = 0; i < all_match_idx.size(); i++) 
                std::cout << "que_idx: " << que_idx - i << ", matcded: " << all_match_idx[i] << std::endl;
        }
        return make_pair(tc_flag, match_idx);
    } 
    else 
    {
        return make_pair(false, match_idx);
    }
}

int main(int argc, char *argv[])
{
    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    LOOP_SKIP_INTERVAL = fsSettings["loop_skip_interval"];
    LOOP_HISTORY_SEARCH_NUM = fsSettings["loop_history_search_num"];
    LOOP_DISTANCE_THRESHOLD = fsSettings["loop_distance_threshold"];
    // LOOP_OPTI_COST_THRESHOLD = fsSettings["loop_opti_cost_threshold"];
    LOOP_TEMPORAL_CONSISTENCY_THRESHOLD = fsSettings["loop_temporal_consistency_threshold"];
    LOOP_GEOMETRIC_CONSISTENCY_THRESHOLD = fsSettings["loop_geometric_consistency_threshold"];

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

    sc_manager.setParameter(LIDAR_HEIGHT,
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

    // ******************************************************
    std::string ref_data_path = "/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_loop/data_2/ref/";
    for (const auto &entry : boost::filesystem::directory_iterator(ref_data_path))
    {
        ref_filename.push_back(std::string(entry.path().c_str()));
    }
    std::sort(ref_filename.begin(), ref_filename.end());
    
    pcl::PointCloud<pcl::PointXYZI> ref, ref_non_ground;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> ref_list;
    for (size_t i = 0; i < ref_filename.size(); i++)
    {
        // std::cout << ref_filename[i] << std::endl;
        pcl::io::loadPCDFile(ref_filename[i].c_str(), ref);
        sc_manager.makeAndSaveScancontextAndKeys(ref);
        ref_list.push_back(ref);
    }
    std::cout << std::endl;

    // ******************************************************
    std::string target_data_path = "/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_loop/data_2/target/";
    for (const auto &entry : boost::filesystem::directory_iterator(target_data_path))
    {
        target_filename.push_back(std::string(entry.path().c_str()));
    }
    std::sort(target_filename.begin(), target_filename.end());
    pcl::PointCloud<pcl::PointXYZI> target, target_non_ground;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> target_list;
    // for (size_t i = 0; i < target_filename.size(); i++)
    for (size_t i = 0; i < target_filename.size(); i = i + 2)
    {
        std::cout << "\nloading: " << target_filename[i] << std::endl;
        pcl::io::loadPCDFile(target_filename[i].c_str(), target);
        target_list.push_back(target);

        TicToc t_make_sc;
        sc_manager.makeAndSaveScancontextAndKeys(target);
        // printf("making sc costs: %fms\n", t_make_sc.toc()); // 0.8ms

        TicToc t_loop_detect;
        auto tc_result = checkTemporalConsistency();
        // printf("loop detection cost: %fms\n\n", t_loop_detect.toc()); // 0.8ms

        if (tc_result.first)
        {
            stringstream ss;
            ss << "/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_loop/data_2/" 
               << "target_" << i << ".pcd";
            pcl::io::savePCDFileASCII(ss.str(), target);
            ss.str("");
            ss << "/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_loop/data_2/"
               << "target_" << i << "_ref_" << tc_result.second << ".pcd";
            pcl::io::savePCDFileASCII(ss.str(), ref_list[tc_result.second]);
        }
    }
    // std::cout << "match id: " << detect_result.first 
    //           << ", yaw: " << detect_result.second << "deg" << std::endl;
    return 0;
}