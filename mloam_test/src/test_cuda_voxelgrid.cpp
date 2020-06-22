#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/cuda/filters/voxel_grid.h>

#include "../src/utility/tic_toc.h"

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    TicToc t_input;
    pcl::PCDReader reader;
    std::string filename = "/home/jjiao/catkin_ws/src/localization/rpg_trajectory_evaluation/results/real_vehicle/shandong/rv02/mloam_map.pcd"; 
    reader.read(filename.c_str(), *cloud); // Remember to download the file first!
    printf("input: %fms\n", t_input.toc());

    {
        TicToc t_filter;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.4f, 0.4f, 0.4f);
        sor.filter(*cloud_filtered);
        printf("no cuda: filter: %fms\n", t_filter.toc());
        pcl::PCDWriter writer;
        writer.writeBinary("mloam_map_ds_nocuda.pcd", *cloud_filtered);
    }

    // {
    //     TicToc t_filter;
    //     pcl_cuda::VoxelGrid<pcl::PointXYZ> sor;
    //     sor.setInputCloud(cloud);
    //     sor.setLeafSize(0.4f, 0.4f, 0.4f);
    //     sor.filter(*cloud_filtered);
    //     printf("cuda: filter: %fms\n", t_filter.toc());
    //     pcl::PCDWriter writer;
    //     writer.writeBinary("mloam_map_ds_cuda.pcd", *cloud_filtered);
    // }

    return (0);
}