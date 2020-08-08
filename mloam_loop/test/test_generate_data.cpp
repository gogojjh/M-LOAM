// rosrun mloam_loop test_generate_data baseline_data/

#include <iostream>
#include <string>

#include <ceres/ceres.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <boost/filesystem.hpp>

Eigen::Matrix4f ReadTrans(const char *filename)
{
    Eigen::Matrix4f temp;
    temp.fill(0);
    int cnt = 0;
    FILE *fid = fopen(filename, "r");
    for (int j = 0; j < 4; j++)
    {
        float a, b, c, d;
        fscanf(fid, "%f %f %f %f", &a, &b, &c, &d);
        temp(j, 0) = a;
        temp(j, 1) = b;
        temp(j, 2) = c;
        temp(j, 3) = d;
    }
    return temp;
}

void WriteTrans(const std::string filepath, Eigen::Matrix4f transtemp)
{
    FILE *fid = fopen(filepath.c_str(), "w");
    fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(0, 0), transtemp(0, 1), transtemp(0, 2), transtemp(0, 3));
    fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(1, 0), transtemp(1, 1), transtemp(1, 2), transtemp(1, 3));
    fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(2, 0), transtemp(2, 1), transtemp(2, 2), transtemp(2, 3));
    fprintf(fid, "%.10f %.10f %.10f %.10f\n", 0.0f, 0.0f, 0.0f, 1.0f);
    fclose(fid);
}

int main(int argc, char* argv[])
{
    pcl::PCDReader pcd_reader;
    pcl::PCDWriter pcd_writer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_map(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcd_reader.read(std::string(argv[1]) + "baseline_data/model.pcd", *laser_map);
    pcd_reader.read(std::string(argv[1]) + "baseline_data/data.pcd", *laser_cloud);
    Eigen::MatrixXf T_ini = ReadTrans(std::string(std::string(argv[1]) + "baseline_data/output_icp.txt").c_str());
    // std::cout << T_ini << std::endl;
    for (int x = 0; x <= 20; x += 5)
    {
        for (int y = 0; y <= 20; y += 5)
        {
            for (int yaw = 0; yaw <= 90; yaw += 30)
            {
                std::stringstream ss;
                ss << std::string(argv[1]) << "transform_data_" << x << "_" << y << "_" << yaw << "/";
                if (!boost::filesystem::exists(ss.str().c_str()))
                    boost::filesystem::create_directory(ss.str().c_str());
                Eigen::Vector3f t(x, y, 0);
                Eigen::Quaternionf q;
                q = Eigen::AngleAxisf(yaw * 3.1415926 / 180.0, Eigen::Vector3f::UnitZ());
                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                T.block<3, 3>(0, 0) = q.toRotationMatrix();
                T.block<3, 1>(0, 3) = t;
                pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_trans(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::transformPointCloud(*laser_cloud, *laser_cloud_trans, T);
                pcd_writer.write(ss.str() + "model.pcd", *laser_map);
                pcd_writer.write(ss.str() + "data.pcd", *laser_cloud_trans);
                Eigen::MatrixXf T_gt = T_ini * T.inverse();
                WriteTrans(std::string(ss.str() + "gt.log").c_str(), T_gt);
                // std::cout << T_gt << std::endl << std::endl;
            }
        }
    }
    printf("Finish generating transform_data !\n");
    return 0;
}

