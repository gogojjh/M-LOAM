#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/transforms.h>

#include <iostream>

using namespace std;

double normal_radius = 1;
double fpfh_radius = 1.5;
double yaw = 3.14;

int main(int argc, char* argv[])
{
    pcl::PCDReader pcd_reader;
    pcl::PCDWriter pcd_writer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcd_reader.read(std::string(argv[1]) + ".pcd", *cloud);
    
    // Eigen::Quaternionf q;
    // q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    // Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    // T.block<3, 3>(0, 0) = q.toRotationMatrix();
    // pcl::transformPointCloud(*cloud, *cloud, T);
    // printf("%lu\n", cloud->size());

    TicToc t_normal;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree_normal);
    // ne.setKSearch(10);
    ne.setRadiusSearch(normal_radius);
    ne.compute(*normals);
    // printf("normal %fms\n", t_normal.toc());

    TicToc t_fpfh;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh(new pcl::search::KdTree<pcl::PointXYZ>());
    fest.setInputCloud(cloud);
    fest.setInputNormals(normals);
    fest.setSearchMethod(tree_fpfh);
    fest.setRadiusSearch(fpfh_radius);
    fest.compute(*object_features);
    // printf("fpfh %fms\n", t_fpfh.toc());

    FILE *fid = fopen(std::string(std::string(argv[1]) + ".bin").c_str(), "wb");
    int nV = cloud->size();
    int nDim = 33;
    fwrite(&nV, sizeof(int), 1, fid);
    fwrite(&nDim, sizeof(int), 1, fid);
    for (int v = 0; v < nV; v++)
    {
        const pcl::PointXYZ &pt = cloud->points[v];
        float xyz[3] = {pt.x, pt.y, pt.z};
        fwrite(xyz, sizeof(float), 3, fid);
        const pcl::FPFHSignature33 &feature = object_features->points[v];
        fwrite(feature.histogram, sizeof(float), 33, fid);
    }
    fclose(fid);
}