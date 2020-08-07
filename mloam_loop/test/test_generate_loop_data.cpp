#include <iostream>
#include <string>

#include <ceres/ceres.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <boost/filesystem.hpp>

#define DEG_TO_RAD 3.1415926 / 180.0;

int main(int argc, char* argv[])
{
    pcl::PCDReader pcd_reader;
    pcl::PCDWriter pcd_writer;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_map(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcd_reader.read(std::string(argv[1]) + "baseline_data/model.pcd", *laser_map);
    pcd_reader.read(std::string(argv[1]) + "baseline_data/data.pcd", *laser_cloud);
    for (double x = 0; x < 20; x += 5)
    {
        for (double y = 0; y < 20; y += 5)
        {
            for (double yaw = 0; yaw < 30; yaw += 5)
            {
                stringstream ss;
                ss << std::string(argv[1]) << "transform_data_" << x << "_" << y << "_" << yaw << "/";
                boost::filesystem::create_directory(ss.str().c_str());
                Eigen::Vector3f t(x, y, 0);
                Eigen::Quaternionf q;
                q = Eigen::AngleAxisf(yaw * DEG_TO_RAD, Eigen::Vector3f::UnitZ());
                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                T.block<3, 3>(0, 0) = q.toRotationMatrix();
                T.block<3, 1>(0, 3) = t;
                pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_trans(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::transformPointCloud(*laser_cloud, laser_cloud_trans, T);
                pcd_writer.write(ss.str() + "model.pcd", *laser_map);
                pcd_writer.write(ss.str() + "data.pcd", *laser_cloud_trans);
            }
        }
    }
    printf("Finish generating transform_data !\n");
    return 0;
}