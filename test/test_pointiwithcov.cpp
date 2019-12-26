#include <eigen3/Eigen/Dense>

#include "mloam_pcl/point_with_cov.h"
#include "../estimator/src/utility/utility.h"

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    int ret = pcl::io::loadPCDFile ("/home/jjiao/catkin_ws/src/localization/M-LOAM/test/left.pcd", *cloud);

    common::PointICovCloud::Ptr cloud_cov(new common::PointICovCloud);
    for (size_t i = 0; i < 100; i++)
    {
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
        cov <<  1.02, 2.0, 3.0, 2.0, 2.1312, 1.0, 3.0, 1.0, 3.2323;
        pcl::PointIWithCov point_cov(cloud->points[i], cov.cast<float>());
        cloud_cov->push_back(point_cov);
    }
    std::cout << cloud_cov->size() << std::endl;

    common::PointICovCloud::Ptr cloud_cov_filter(new common::PointICovCloud);
    pcl::VoxelGrid<pcl::PointIWithCov> down_size_filter;
    down_size_filter.setLeafSize(0.2, 0.2, 0.2);
    down_size_filter.setInputCloud(cloud_cov);
    down_size_filter.filter(*cloud_cov_filter);
    std::cout << cloud_cov_filter->size() << std::endl;
    for (auto p: cloud_cov_filter->points)
    {
        std::cout << p;
    }

    pcl::KdTreeFLANN<pcl::PointIWithCov>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointIWithCov>());
    kdtree->setInputCloud(cloud_cov);

    return 0;
}



//
