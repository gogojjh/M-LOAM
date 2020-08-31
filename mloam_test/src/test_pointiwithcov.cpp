// Starting with PCL-1.7 you need to define PCL_NO_PRECOMPILE before you include any PCL headers to include the templated algorithms as well.
#define PCL_NO_PRECOMPILE

#include "mloam_pcl/point_with_cov.hpp"
#include "mloam_pcl/voxel_grid_covariance_mloam.h"
#include "mloam_pcl/voxel_grid_covariance_mloam_impl.hpp"

// #include <pcl/common/centroid.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/impl/filter.hpp>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/impl/voxel_grid.hpp>

#include <eigen3/Eigen/Dense>

typedef pcl::PointXYZIWithCov PointType;
typedef pcl::PointCloud<PointType> PointCloud;

int main()
{
    std::cout << "Testing pointiwithcov.cpp" << std::endl;
    pcl::PointCloud<PointType>::Ptr cloud_cov(new pcl::PointCloud<PointType>);
    PointType p1(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    PointType p2(1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    PointType p3(0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
    PointType p4(1, 1, 0, 0, 1, 0, 0, 0, 0, 0);
    cloud_cov->push_back(p1);
    cloud_cov->push_back(p2);
    cloud_cov->push_back(p3);
    cloud_cov->push_back(p4);
    for (auto p:cloud_cov->points) std::cout << p << std::endl;

    pcl::PointCloud<PointType>::Ptr cloud_cov_filter(new pcl::PointCloud<PointType>);
    pcl::VoxelGridCovarianceMLOAM<PointType> down_size_filter;
    down_size_filter.setInputCloud(cloud_cov);
    down_size_filter.setTraceThreshold(2);
    down_size_filter.setLeafSize(3, 3, 3);
    down_size_filter.filter(*cloud_cov_filter);
    std::cout << cloud_cov_filter->size() << std::endl;
    for (auto p:cloud_cov_filter->points) std::cout << p << std::endl;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
    kdtree->setInputCloud(cloud_cov);
    return 0;
}



//
