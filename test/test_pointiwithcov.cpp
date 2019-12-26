// Starting with PCL-1.7 you need to define PCL_NO_PRECOMPILE before you include any PCL headers to include the templated algorithms as well.
#define PCL_NO_PRECOMPILE

#include "mloam_pcl/point_with_cov.h"
#include "mloam_pcl/voxel_grid_covariance_mloam.h"
#include "mloam_pcl/voxel_grid_covariance_mloam_impl.hpp"

#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

#include <eigen3/Eigen/Dense>

#include "../estimator/src/utility/utility.h"

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile ("/home/jjiao/catkin_ws/src/localization/M-LOAM/test/left.pcd", *cloud);
    pcl::PointCloud<pcl::PointIWithCov>::Ptr cloud_cov(new pcl::PointCloud<pcl::PointIWithCov>);
    for (size_t i = 0; i < 100; i++)
    {
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
        cov <<  1.02, 2.0, 3.0, 2.0, 2.1312, 1.0, 3.0, 1.0, 3.2323;
        pcl::PointIWithCov point_cov(cloud->points[i], cov.cast<float>());
        cloud_cov->push_back(point_cov);
    }
    std::cout << cloud_cov->size() << std::endl;

    pcl::PointCloud<pcl::PointIWithCov>::Ptr cloud_cov_filter(new pcl::PointCloud<pcl::PointIWithCov>);
    pcl::VoxelGridCovarianceMLOAM<pcl::PointIWithCov> down_size_filter;
    down_size_filter.setInputCloud(cloud_cov);
    down_size_filter.setLeafSize(0.2, 0.2, 0.2);
    down_size_filter.filter(*cloud_cov_filter);
    std::cout << cloud_cov_filter->size() << std::endl;

//
//     pcl::CentroidPoint<pcl::PointIWithCov> centroid;
//     centroid.add(pcl::PointIWithCov(1, 2, 3, 4, 1, 2, 3, 4, 5, 6));
//     centroid.add(pcl::PointIWithCov(7, 8, 9, 10, 7, 8, 9, 10, 11, 12));
//     pcl::PointIWithCov c1;
//     centroid.get(c1);
//     std::cout << c1;
//
//     std::cout << pcl::traits::has_field<pcl::PointIWithCov, pcl::fields::cov_xx>::value << std::endl;
//
//     common::PointICovCloud::Ptr cloud_cov_filter(new common::PointICovCloud);
//     pcl::VoxelGrid<pcl::PointIWithCov> down_size_filter;
//     down_size_filter.setInputCloud(cloud_cov);
// //    std::cout << down_size_filter.getFilterFieldName() << std::endl;
// //    down_size_filter.setFilterFieldName("cov_xx");
// //    std::cout << down_size_filter.getFilterFieldName() << std::endl;
//
//     down_size_filter.setLeafSize(0.2, 0.2, 0.2);
//     down_size_filter.filter(*cloud_cov_filter);
//     std::cout << cloud_cov_filter->size() << std::endl;
// //    for (auto p: cloud_cov_filter->points)
// //    {
// //        std::cout << p;
// //    }
//
//     pcl::KdTreeFLANN<pcl::PointIWithCov>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointIWithCov>());
//     kdtree->setInputCloud(cloud_cov);

    return 0;
}



//
