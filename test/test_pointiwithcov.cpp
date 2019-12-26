// Starting with PCL-1.7 you need to define PCL_NO_PRECOMPILE before you include any PCL headers to include the templated algorithms as well.
#define PCL_NO_PRECOMPILE

#include "mloam_pcl/point_with_cov.h"
#include "mloam_pcl/voxel_grid_covariance_mloam.h"
#include "mloam_pcl/voxel_grid_covariance_mloam_impl.hpp"

// #include <pcl/common/centroid.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/impl/filter.hpp>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/impl/voxel_grid.hpp>

#include <eigen3/Eigen/Dense>

#include "../estimator/src/utility/utility.h"

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile ("/home/jjiao/catkin_ws/src/localization/M-LOAM/test/left.pcd", *cloud);
    pcl::PointCloud<pcl::PointIWithCov>::Ptr cloud_cov(new pcl::PointCloud<pcl::PointIWithCov>);
    // for (size_t i = 0; i < 100; i++)
    // {
    //     Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    //     cov <<  1.02, 2.0, 3.0, 2.0, 2.1312, 1.0, 3.0, 1.0, 3.2323;
    //     pcl::PointIWithCov point_cov(cloud->points[i], cov.cast<float>());
    //     cloud_cov->push_back(point_cov);
    // }
    // std::cout << cloud_cov->size() << std::endl;

    pcl::PointIWithCov p1(0,0,0,0,0.0025,0,0,0.0025,0,0.0025);
    pcl::PointIWithCov p2(1,1,1,1,0.6825, -0.08, -0.12, 0.5625, -0.24, 0.3625);
    pcl::PointIWithCov p3(2,2,2,2,2.085, -0.245, -0.3675, 1.7175, -0.735, 1.105);
    cloud_cov->push_back(p1);
    cloud_cov->push_back(p2);
    cloud_cov->push_back(p3);
    for (auto p:cloud_cov->points) std::cout << p << std::endl;

    pcl::PointCloud<pcl::PointIWithCov>::Ptr cloud_cov_filter(new pcl::PointCloud<pcl::PointIWithCov>);
    pcl::VoxelGridCovarianceMLOAM<pcl::PointIWithCov> down_size_filter;
    down_size_filter.setInputCloud(cloud_cov);
    down_size_filter.setLeafSize(3, 3, 3);
    down_size_filter.filter(*cloud_cov_filter);
    std::cout << cloud_cov_filter->size() << std::endl;
    for (auto p:cloud_cov_filter->points) std::cout << p << std::endl;

    // pcl::KdTreeFLANN<pcl::PointIWithCov>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointIWithCov>());
    // kdtree->setInputCloud(cloud_cov);

    return 0;
}



//
