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

#include "mloam_loop/tic_toc.h"
#include "mloam_loop/parameters.h"
#include "mloam_loop/scan_context.hpp"

using namespace std;

void ground_filter(const pcl::PointCloud<pcl::PointXYZI> &pc_in, pcl::PointCloud<pcl::PointXYZI> &pc_out)
{
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(pc_in));
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-3.0, 50.0);
    pass.filter(pc_out);
}

int main(int argc, char *argv[])
{
    readParameters(std::string(argv[1]).c_str());

    SCManager sc_manager;
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

    std::string ref_data_path = "/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_loop/data/ref/";
    std::vector<std::string> ref_filename;
    for (const auto &entry : boost::filesystem::directory_iterator(ref_data_path))
    {
        ref_filename.push_back(std::string(entry.path().c_str()));
    }
    std::sort(ref_filename.begin(), ref_filename.end());
    pcl::PointCloud<pcl::PointXYZI> ref, ref_non_ground;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> ref_list;
    for (size_t i = 0; i < ref_filename.size(); i++)
    {
        // std::cout << "loading: " << ref_filename[i] << std::endl;
        pcl::io::loadPCDFile(ref_filename[i].c_str(), ref);
        // ground_filter(ref, ref_non_ground);
        // if (i == 0) 
        //     pcl::io::savePCDFileASCII("/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_loop/data/test.pcd", ref_non_ground);
        sc_manager.makeAndSaveScancontextAndKeys(ref);
        ref_list.push_back(ref);
    }
    std::cout << std::endl;

    std::string target_data_path = "/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_loop/data/target/";
    std::vector<std::string> target_filename;
    for (const auto &entry : boost::filesystem::directory_iterator(target_data_path))
    {
        // std::cout << "loading: " << entry.path() << std::endl;
        target_filename.push_back(std::string(entry.path().c_str()));
    }
    std::sort(target_filename.begin(), target_filename.end());
    pcl::PointCloud<pcl::PointXYZI> target, target_non_ground;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> target_list;
    for (size_t i = 0; i < target_filename.size(); i++)
    {
        TicToc t_loop_detect;
        std::cout << "loading: " << target_filename[i] << std::endl;
        pcl::io::loadPCDFile(target_filename[i].c_str(), target);
        // ground_filter(target, target_non_ground);
        sc_manager.makeAndSaveScancontextAndKeys(target);
        target_list.push_back(target);
        auto detect_result = sc_manager.detectLoopClosureID();
        printf("loop detection cost: %fms\n\n", t_loop_detect.toc());
        if (detect_result.first != -1)
        {
            int match_id = detect_result.first;
            stringstream ss;
            ss << "/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_loop/data/" 
               << "target_" << i << ".pcd";
            pcl::io::savePCDFileASCII(ss.str(), target);
            ss.str("");
            ss << "/home/jjiao/catkin_ws/src/localization/M-LOAM/mloam_loop/data/"
               << "target_" << i << "_ref_" << match_id << ".pcd";
            pcl::io::savePCDFileASCII(ss.str(), ref_list[match_id]);
        }
    }
    // std::cout << "match id: " << detect_result.first 
    //           << ", yaw: " << detect_result.second << "deg" << std::endl;
    return 0;
}