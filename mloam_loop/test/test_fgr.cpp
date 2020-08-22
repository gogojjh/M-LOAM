// rosrun mloam_loop test_fgr baseline_data/

#include <iostream>
#include <string>

#include <ceres/ceres.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/features/fpfh_omp.h>

#include "mloam_loop/utility/tic_toc.h"
#include "../ThirdParty/FastGlobalRegistration/app.h"

#define DIV_FACTOR			1.4		// Division factor used for graduated non-convexity
#define USE_ABSOLUTE_SCALE	1		// Measure distance in absolute scale (1) or in scale relative to the diameter of the model (0)
#define MAX_CORR_DIST		0.025	// Maximum correspondence distance (also see comment of USE_ABSOLUTE_SCALE)
#define ITERATION_NUMBER	64		// Maximum number of iteration
#define TUPLE_SCALE			0.95	// Similarity measure used for tuples of feature points.
#define TUPLE_MAX_CNT		1000	// Maximum tuple numbers.

double normal_radius = 1;
double fpfh_radius = 1.5;

void parseFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
               const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfh_feature, 
               fgr::Points &points, 
               fgr::Feature &features)
{
    for (size_t i = 0; i < cloud->size(); i++)
    {
        const pcl::PointXYZ &pt = cloud->points[i];
        Eigen::Vector3f pts(pt.x, pt.y, pt.z);
        points.push_back(pts);

        pcl::FPFHSignature33 feature = fpfh_feature->points[i];
        Eigen::VectorXf feat(33);
        for (size_t j = 0; j < 33; j++) feat(j) = feature.histogram[j];
        features.push_back(feat);
    }
}

void WriteTime(const char *filepath, double time)
{
    FILE *fid = fopen(filepath, "w");
    fprintf(fid, "%.10f\n", time);
    printf("time: %.10fms\n", time);
    fclose(fid);
}

int main(int argc, char *argv[])
{
    pcl::PCDReader pcd_reader;
    pcl::PCDWriter pcd_writer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_map(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcd_reader.read(std::string(argv[1]) + "model.pcd", *laser_map);
    pcd_reader.read(std::string(argv[1]) + "data.pcd", *laser_cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_cloud;
    point_cloud.push_back(laser_map);
    point_cloud.push_back(laser_cloud);

    double time = 0.0;
    TicToc t_fpfh;
    std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_feature;
    for (size_t i = 0; i < point_cloud.size(); i++)
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setInputCloud(point_cloud[i]);
        ne.setSearchMethod(tree_normal);
        // ne.setKSearch(10);
        ne.setRadiusSearch(normal_radius);
        ne.compute(*normals);

        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh(new pcl::search::KdTree<pcl::PointXYZ>());
        fest.setInputCloud(point_cloud[i]);
        fest.setInputNormals(normals);
        fest.setSearchMethod(tree_fpfh);
        fest.setRadiusSearch(fpfh_radius);
        fest.compute(*object_features);
        fpfh_feature.push_back(object_features);
    }
    fgr::Points p1, p2;
    fgr::Feature f1, f2;
    parseFPFH(point_cloud[0], fpfh_feature[0], p1, f1);
    parseFPFH(point_cloud[1], fpfh_feature[1], p2, f2);
    printf("extract fpfh from %lu clouds: %fms\n", fpfh_feature.size(), t_fpfh.toc());
    time += t_fpfh.toc();
    
    TicToc t_fgr;
	fgr::CApp app(DIV_FACTOR, USE_ABSOLUTE_SCALE, MAX_CORR_DIST, ITERATION_NUMBER, TUPLE_SCALE, TUPLE_MAX_CNT);
    app.LoadFeature(p1, f1);
    app.LoadFeature(p2, f2);
	app.NormalizePoints();
	app.AdvancedMatching();
	app.OptimizePairwise(true);
    printf("FGR: %fms\n", t_fgr.toc());
    time += t_fgr.toc();

    Eigen::MatrixXf T = app.GetOutputTrans();
    std::cout << T << std::endl;

    pcl::transformPointCloud(*laser_cloud, *laser_cloud, T);
    pcd_writer.write(std::string(argv[1]) + "data_fgr.pcd", *laser_cloud);
	app.WriteTrans(std::string(std::string(argv[1]) + "output_fgr.txt").c_str());
    app.WriteCost(std::string(std::string(argv[1]) + "cost_fgr.txt").c_str());
    WriteTime(std::string(std::string(argv[1]) + "time_fgr.txt").c_str(), time);
}


