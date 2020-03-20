#pragma once

#include <thread>
#include <mutex>
#include <chrono>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <map>

#include "common/types/type.h"

class PlaneNormalVisualizer
{
public:
    PlaneNormalVisualizer();
    void Spin();
    void UpdateCloud(common::PointCloud::ConstPtr cloud,
                       std::string cloud_name = "cloud",
                       std::vector<double> cloud_color = {1.0, 0.0, 1.0});

    void UpdateCloudAndNormals(common::PointCloud::ConstPtr cloud,
                                 common::NormalCloud::ConstPtr normals,
                                 int ds_ratio = 10,
                                 std::string cloud_name = "cloud",
                                 std::string normals_name = "normals",
                                 std::vector<double> cloud_color = {1.0, 1.0, 1.0},
                                 std::vector<double> normals_color = {1.0, 1.0, 0.0});

    void UpdateLines(common::PointCloud::ConstPtr cloud1,
                       common::PointCloud::ConstPtr cloud2,
                       std::vector<double> line_color = {0.0, 1.0, 0.0});

    void UpdatePlanes(const std::vector<Eigen::Vector4d> &plane_coeffs);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    //  pcl::visualization::PCLVisualizer* viewer;
    std::mutex m_vis_;
    bool init_ = false;
    bool first_ = false;

    std::vector<std::string> line_names_;
    std::vector<std::string> plane_names_;
};
