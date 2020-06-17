// how to use: 
// rosrun mloam test_generate_bag_from_data_oxford 
//   -data_path=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common/common.hpp"
#include "common/publisher.hpp"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

// PCL
#include <pcl/point_cloud.h> /* pcl::PointCloud */
#include <pcl/point_types.h> /* pcl::PointXYZ */
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include "../src/estimator/pose.h"

#include <iostream>
#include <iomanip>

using namespace std;

DEFINE_string(data_path, "/", "oxford data path");

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]), num_elements * sizeof(float));
    return lidar_data_buffer;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_merge_pointcloud");
    ros::NodeHandle nh("~");

    google::ParseCommandLineFlags(&argc, &argv, true);
    string data_path = FLAGS_data_path;
    std::cout << data_path << std::endl;

    rosbag::Bag bag;
    bag.open((data_path + "data.bag").c_str(), rosbag::bagmode::Write);

    unsigned long long timestamp;
    double start_time = 1e15;
    double end_time = 0;
    int id;
    FILE *file;
    pcl::PCDReader pcd_reader;
    size_t frame_cnt;

    frame_cnt = 0;
    file = std::fopen((data_path + "velodyne_left.timestamps").c_str(), "r");
    while (fscanf(file, "%llu %d", &timestamp, &id) != EOF)
    {
        stringstream ss;
        ss << data_path << "velodyne_left/" << timestamp << ".pcd";
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        pcd_reader.read(ss.str(), laser_cloud);

        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(laser_cloud, msg_cloud);
        std_msgs::Header header;
        header.stamp = ros::Time(timestamp * 1.0 / 1e6);
        header.frame_id = "/velo_left";
        msg_cloud.header = header;
        bag.write("/left/velodyne_points", ros::Time(timestamp * 1.0 / 1e6), msg_cloud);
        if (timestamp * 1.0 / 1e6 < start_time)
            start_time = timestamp * 1.0 / 1e6;
        if (timestamp * 1.0 / 1e6 > end_time)
            end_time = timestamp * 1.0 / 1e6;
        frame_cnt++;
        // if (frame_cnt > 100) break;
        // std::cout << timestamp << std::endl;
    }
    std::cout << "Finish loading velo_left" << std::endl;

    frame_cnt = 0;
    file = std::fopen((data_path + "velodyne_right.timestamps").c_str(), "r");
    while (fscanf(file, "%llu %d", &timestamp, &id) != EOF)
    {
        stringstream ss;
        ss << data_path << "velodyne_right/" << timestamp << ".pcd";
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        pcd_reader.read(ss.str(), laser_cloud);

        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(laser_cloud, msg_cloud);
        std_msgs::Header header;
        header.stamp = ros::Time(timestamp * 1.0 / 1e6);
        header.frame_id = "/velo_right";
        msg_cloud.header = header;
        bag.write("/right/velodyne_points", ros::Time(timestamp * 1.0 / 1e6), msg_cloud);
        frame_cnt++;
        // if (frame_cnt > 100) break;
        // std::cout << timestamp << std::endl;
    }
    std::cout << "Finish loading velo_right" << std::endl;
 
    frame_cnt = 0;
    file = std::fopen((data_path + "stereo.timestamps").c_str(), "r");
    while (fscanf(file, "%llu %d", &timestamp, &id) != EOF)
    {
        stringstream ss;
        ss << data_path << "stereo/centre/" << timestamp << ".png";

        cv::Mat img;
        img = cv::imread(ss.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
        sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
        imgMsg->header.stamp = ros::Time(timestamp * 1.0 / 1e6);
        bag.write("/stereo/centre", ros::Time(timestamp * 1.0 / 1e6), imgMsg);
        frame_cnt++;
        // if (frame_cnt > 100) break;
        // std::cout << timestamp << std::endl;
    }
    std::cout << "Finish loading stereo" << std::endl;

    std::ifstream ground_truth_file(data_path + "gt/radar_odometry.csv", std::ifstream::in);
    std::string line;
    std::getline(ground_truth_file, line);
    double s_time, d_time, x, y, z, roll, pitch, yaw, s_radar_time, d_radar_time;
    Eigen::Quaterniond q; 
    q.w() = 0;
    q.z() = 1;
    Eigen::Vector3d t(0, 0, 0);
    Pose pose_cur(q, t);

    // std::cout << std::fixed << std::setprecision(6) << start_time << ", " << end_time << std::endl;
    frame_cnt = 0;
    while (std::getline(ground_truth_file, line))
    {
        std::stringstream pose_stream(line);
        std::string s;
        std::vector<double> pose_data;
        while (std::getline(pose_stream, s, ','))
        {
            pose_data.push_back(stod(s));
        }
        s_time = pose_data[0] * 1.0 / 1e6;
        d_time = pose_data[1] * 1.0 / 1e6;
        if (s_time < start_time) continue;
        if (d_time > end_time) continue;
        x = pose_data[2];
        y = -pose_data[3];
        z = -pose_data[4];
        roll = pose_data[5];
        pitch = pose_data[6];
        yaw = -pose_data[7];
        Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Vector3d::UnitX()) 
                             * Eigen::AngleAxisd(pitch, Vector3d::UnitY()) 
                             * Eigen::AngleAxisd(yaw, Vector3d::UnitZ());
        Eigen::Vector3d t(x, y, z);
        Pose pose_local(q, t);
        pose_cur = pose_cur * pose_local;

        nav_msgs::Odometry odomGT;
        odomGT.header.stamp = ros::Time(d_time);
        odomGT.header.frame_id = "/world";
        odomGT.child_frame_id = "/gt";
        odomGT.pose.pose.orientation.x = pose_cur.q_.x();
        odomGT.pose.pose.orientation.y = pose_cur.q_.y();
        odomGT.pose.pose.orientation.z = pose_cur.q_.z();
        odomGT.pose.pose.orientation.w = pose_cur.q_.w();
        odomGT.pose.pose.position.x = pose_cur.t_(0);
        odomGT.pose.pose.position.y = pose_cur.t_(1);
        odomGT.pose.pose.position.z = pose_cur.t_(2);
        bag.write("/current_odom", ros::Time(d_time), odomGT);
        frame_cnt++;
        // if (frame_cnt > 100) break;
    }
    std::cout << "Finish loading gt" << std::endl;

    bag.close();
    return 0;
}
