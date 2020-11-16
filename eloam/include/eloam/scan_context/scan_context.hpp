#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.hpp"
#include "../utility/tic_toc.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType = pcl::PointXYZI; // using xyz only. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

class QueryResult
{
public:
    QueryResult(int match_index,
                double score,
                double yaw_diff_rad) : match_index_(match_index),
                                       score_(score),
                                       yaw_diff_rad_(yaw_diff_rad) {}
    friend std::ostream &operator<<(std::ostream &out, const QueryResult &qr);
    int match_index_;
    double score_;
    double yaw_diff_rad_;
};

void coreImportTest(void);
float xy2theta(const float &_x, const float &_y);
float rad2deg(const float radians);
float deg2rad(const float degrees);

MatrixXd circshift(MatrixXd &_mat, int _num_shift);
std::vector<float> eig2stdvec(MatrixXd _eigmat);

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) 
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

class SCManager
{
public:
    SCManager()
    {

    }

    void setParameter(double lidar_height,
                      int pc_num_ring,
                      int pc_num_sector,
                      double pc_max_radius,
                      double pc_unit_sectorangle,
                      double pc_unit_ringgap,
                      int num_exclude_recent,
                      int num_candidates_from_tree,
                      double search_ratio,
                      double sc_dist_thres,
                      int tree_making_period) 
    {                    
        LIDAR_HEIGHT = lidar_height;
        PC_NUM_RING = pc_num_ring;
        PC_NUM_SECTOR = pc_num_sector;
        PC_MAX_RADIUS = pc_max_radius;
        PC_UNIT_SECTORANGLE = pc_unit_sectorangle;
        PC_UNIT_RINGGAP = pc_unit_ringgap;
        NUM_EXCLUDE_RECENT = num_exclude_recent;
        NUM_CANDIDATES_FROM_TREE = num_candidates_from_tree;
        SEARCH_RATIO = search_ratio;
        SC_DIST_THRES = sc_dist_thres;
        TREE_MAKING_PERIOD = tree_making_period;

        std::cout << "[SCManager param]: "
                  << "lidar_height: " << LIDAR_HEIGHT << ", " 
                  << "pc_num_ring: " << PC_NUM_RING << ", " 
                  << "pc_num_sector: " << PC_NUM_SECTOR << ", " 
                  << "pc_max_radius: " << PC_MAX_RADIUS << ", " 
                  << "pc_unit_sectorangle: " << PC_UNIT_SECTORANGLE << ", " 
                  << "pc_unit_ringgap: " << PC_UNIT_RINGGAP << ", " 
                  << "num_exclude_recent: " << NUM_EXCLUDE_RECENT << ", " 
                  << "num_candidates_from_tree: " << NUM_CANDIDATES_FROM_TREE << ", " 
                  << "search_ratio: " << SEARCH_RATIO << ", " 
                  << "sc_dist_thres: " << SC_DIST_THRES << ", " 
                  << "tree_making_period: " << TREE_MAKING_PERIOD << std::endl;

        tree_making_period_conter_ = 0;

        init_color();
    }        

    Eigen::MatrixXd makeScancontext(pcl::PointCloud<SCPointType> & _scan_down);
    Eigen::MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd &_desc);
    Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc);

    int fastAlignUsingVkey(MatrixXd &_vkey1, MatrixXd &_vkey2);
    double distDirectSC(MatrixXd &_sc1, MatrixXd &_sc2);                           // "d" (eq 5) in the original paper (IROS 18)
    std::pair<double, int> distanceBtnScanContext(MatrixXd &_sc1, MatrixXd &_sc2); // "D" (eq 6) in the original paper (IROS 18)

    // User-side API
    void makeAndSaveScancontextAndKeys(pcl::PointCloud<SCPointType> &_scan_down);
    QueryResult detectLoopClosureID(const int &query_idx); // int: nearest node index, float: relative yaw

    void init_color();
    cv::Mat getScanContextImage(const int &que_index);
    size_t getDataBaseSize();

public:
    // hyper parameters ()
    double LIDAR_HEIGHT; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

    int PC_NUM_RING;        // 20 in the original paper (IROS 18)
    int PC_NUM_SECTOR;      // 60 in the original paper (IROS 18)
    double PC_MAX_RADIUS;   // 80 meter max in the original paper (IROS 18)
    double PC_UNIT_SECTORANGLE;
    double PC_UNIT_RINGGAP;

    // tree
    int NUM_EXCLUDE_RECENT;       // simply just keyframe gap, but node position distance-based exclusion is ok.
    int NUM_CANDIDATES_FROM_TREE; // 10 is enough. (refer the IROS 18 paper)

    // loop thres
    double SEARCH_RATIO; // for fast comparison, no Brute-force, but search 10 % is okay. // but may well work for same-direction-revisits, not for reverse-revisits
    // double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    double SC_DIST_THRES; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold

    // config
    int TREE_MAKING_PERIOD; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost
    int tree_making_period_conter_;

    // data
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

    std::vector<cv::Vec3b> color_projection_;

}; // SCManager

// } // namespace SC2