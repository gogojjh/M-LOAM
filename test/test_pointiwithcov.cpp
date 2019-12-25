#include <eigen3/Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "common/types/type.h"
#include "common/types/point_with_cov.h"
#include "../estimator/src/utility/utility.h"

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    int ret = pcl::io::loadPCDFile ("/home/jjiao/catkin_ws/src/localization/M-LOAM/test/left.pcd", *cloud);

    common::PointICovCloud::Ptr cloud_cov(new common::PointICovCloud);
    for (size_t i = 0; i < 100; i++)
    {
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        pcl::PointIWithCov point_cov(cloud->points[i], cov.cast<float>());
        cloud_cov->push_back(point_cov);
    }
    std::cout << cloud_cov->size() << std::endl;

    common::PointICovCloud::Ptr cloud_cov_filter(new common::PointICovCloud);
    pcl::VoxelGrid<pcl::PointIWithCov> down_size_filter;
    // down_size_filter.setLeafSize(0.2, 0.2, 0.2);
    // down_size_filter.setInputCloud(cloud_cov);
    // down_size_filter.filter(*cloud_cov_filter);
    // std::cout << cloud_cov_filter->size() << std::endl;

    return 0;
}

// #define PCL_NO_PRECOMPILE
// #include <pcl/pcl_macros.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>
//
// namespace pcl
// {
// struct EIGEN_ALIGN16 _PointIWithCov
// {
//     PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
//     float intensity;
//     float cov_vec[6];
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
// } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
//
// struct PointIWithCov: public _PointIWithCov
// {
//
// };
// }
//
// POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointIWithCov,           // here we assume a XYZ + "test" (as fields)
//                                    (float, x, x)
//                                    (float, y, y)
//                                    (float, z, z)
//                                    (float, intensity, intensity)
//                                    (float[6], cov_vec, cov)
// )
// POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointIWithCov, pcl::_PointIWithCov)


// int main (int argc, char** argv)
// {
//   pcl::PointCloud<pcl::PointIWithCov> cloud;
//   cloud.points.resize(2);
//   cloud.width = 2;
//   cloud.height = 1;
//
//   cloud.points[0].x = cloud.points[0].y = cloud.points[0].z = 0;
//   cloud.points[1].x = cloud.points[1].y = cloud.points[1].z = 3;
//   cloud.points[0].intensity = 128;
//
//   pcl::io::savePCDFile ("/home/jjiao/catkin_ws/src/localization/M-LOAM/test/test_pointiwithcov.pcd", cloud);
// }




//
