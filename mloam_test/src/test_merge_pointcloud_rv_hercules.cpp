#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include "common/common.hpp"
#include "common/publisher.hpp"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

// PCL
#include <pcl/point_cloud.h>			/* pcl::PointCloud */
#include <pcl/point_types.h>			/* pcl::PointXYZ */
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace std;

typedef sensor_msgs::PointCloud2 LidarMsgType;
typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType, LidarMsgType, LidarMsgType> LidarSyncPolicy;
typedef message_filters::Subscriber<LidarMsgType> LidarSubType;

ros::Publisher cloud_fused_pub;

Eigen::Quaterniond q_world_ref_ini;
Eigen::Vector3d t_world_ref_ini;
nav_msgs::Path laser_gt_path;

bool b_pause;

string MLOAM_GT_PATH;

int frame_cnt = 0;
size_t DELTA_IDX;

std::vector<Eigen::Matrix4d> TBL;

size_t NUM_OF_LASER;

void pauseCallback(std_msgs::StringConstPtr msg)
{
    printf("%s\n", msg->data.c_str());
    b_pause = !b_pause;
}

Eigen::Matrix4d getTransformMatrix(const std::vector<double>& calib)
{
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
    if (calib.size() == 6)
    {
        double x = calib[4], y = calib[5], z = calib[6];
        Eigen::Vector3d translation(x, y, z);
        trans.topRightCorner<3, 1>() = translation;

        double yaw = calib[0], pitch = calib[1], roll = calib[2];
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d rotation = q.matrix();
        trans.topLeftCorner<3, 3>() = rotation;
    }
    else if (calib.size() == 7)
    {
        double x = calib[4], y = calib[5], z = calib[6];
        Eigen::Vector3d translation(x, y, z);
        trans.topRightCorner<3, 1>() = translation;

        Eigen::Quaternion<double> q(calib[3], calib[0], calib[1], calib[2]);
        Eigen::Matrix3d rotation = q.matrix();
        trans.topLeftCorner<3, 3>() = rotation;
    }
    return trans;
}

void process(const sensor_msgs::PointCloud2ConstPtr& pc2_top,
             const sensor_msgs::PointCloud2ConstPtr& pc2_front,
             const sensor_msgs::PointCloud2ConstPtr& pc2_left,
             const sensor_msgs::PointCloud2ConstPtr& pc2_right)
{
    pcl::PointCloud<pcl::PointXYZI> cloud, cloud_fused;
    std_msgs::Header header = pc2_top->header;
    {
        pcl::fromROSMsg(*pc2_top, cloud);
        pcl::transformPointCloud(cloud, cloud, TBL[0].cast<float>());
        for (auto &p : cloud.points)
            p.intensity = 0;
        cloud_fused += cloud;
        // std::cout << cloud_fused.size() << std::endl;
    }
    if (NUM_OF_LASER >= 2) 
    {
        pcl::fromROSMsg(*pc2_front, cloud);
        pcl::transformPointCloud(cloud, cloud, TBL[1].cast<float>());
        for (auto &p : cloud.points) p.intensity = 1;
        cloud_fused += cloud;
        // std::cout << cloud_fused.size() << std::endl;
    }
    if (NUM_OF_LASER >= 3) 
    {
        pcl::fromROSMsg(*pc2_left, cloud);
        pcl::transformPointCloud(cloud, cloud, TBL[2].cast<float>());
        for (auto &p : cloud.points) p.intensity = 2;
        cloud_fused += cloud;
        // std::cout << cloud_fused.size() << std::endl;
    }
    if (NUM_OF_LASER >= 4) 
    {
        pcl::fromROSMsg(*pc2_right, cloud);
        pcl::transformPointCloud(cloud, cloud, TBL[3].cast<float>());
        for (auto &p : cloud.points) p.intensity = 3;
        cloud_fused += cloud;
        // std::cout << cloud_fused.size() << std::endl;
    }
    if (frame_cnt % DELTA_IDX == 0)
        common::publishCloud(cloud_fused_pub, header, cloud_fused);
    frame_cnt++;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(), "r");
    if (fh == NULL)
    {
        std::cout << "config_file dosen't exist; wrong config_file path" << std::endl;
        ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    int num_of_laser = fsSettings["num_of_laser"];
    assert(num_of_laser >= 0);
    NUM_OF_LASER = (size_t)num_of_laser;
    printf("laser number %lu\n", NUM_OF_LASER);

    TBL.resize(NUM_OF_LASER);
    cv::Mat cv_T;
    fsSettings["body_T_laser"] >> cv_T;
    for (int i = 0; i < NUM_OF_LASER; i++)
    {
        Eigen::Quaterniond q = Eigen::Quaterniond(cv_T.ptr<double>(i)[3], cv_T.ptr<double>(i)[0], cv_T.ptr<double>(i)[1], cv_T.ptr<double>(i)[2]);
        Eigen::Vector3d t = Eigen::Vector3d(cv_T.ptr<double>(i)[4], cv_T.ptr<double>(i)[5], cv_T.ptr<double>(i)[6]);
        TBL[i].setIdentity();
        TBL[i].topLeftCorner<3, 3>() = q.toRotationMatrix();
        TBL[i].topRightCorner<3, 1>() = t;
        std::cout << q.coeffs().transpose() << ", " << t.transpose() << std::endl;
        std::cout << TBL[i] << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_merge_pointcloud_rv_hercules");
    ros::NodeHandle nh("~");
    cloud_fused_pub = nh.advertise<LidarMsgType>("/fused/velodyne_points", 5);
    DELTA_IDX = std::stoi(argv[3]);
    std::string data_source(argv[1]);

    readParameters(std::string(argv[6]));

    if (!data_source.compare("bag"))
    {
        //register fusion callback function
        LidarSubType *sub_top = new LidarSubType(nh, "/top/rslidar_points", 1);
        LidarSubType *sub_front = new LidarSubType(nh, "/front/rslidar_points", 1);
        LidarSubType *sub_left = new LidarSubType(nh, "/left/rslidar_points", 1);
        LidarSubType *sub_right = new LidarSubType(nh, "/right/rslidar_points", 1);
        message_filters::Synchronizer<LidarSyncPolicy> *lidar_synchronizer =
            new message_filters::Synchronizer<LidarSyncPolicy>(
                LidarSyncPolicy(10), *sub_top, *sub_front, *sub_left, *sub_right);
        lidar_synchronizer->registerCallback(boost::bind(&process, _1, _2, _3, _4));

        ros::Rate fps(100);
        while (ros::ok())
        {
            ros::spinOnce();
            fps.sleep();
        }
    } 
    else if (!data_source.compare("pcd"))
    {
        std::string data_path = argv[2];
        ros::Subscriber sub_pause = nh.subscribe<std_msgs::String>("/mloam_pause", 5, pauseCallback);
        ros::Publisher pub_laser_gt_odom = nh.advertise<nav_msgs::Odometry>("/laser_gt_odom", 10);
        ros::Publisher pub_laser_gt_path = nh.advertise<nav_msgs::Path>("/laser_gt_path", 10);
        ros::Publisher pub_gps = nh.advertise<sensor_msgs::NavSatFix>("/novatel718d/pos", 10);
        // ros::Publisher pub_gps_odom = nh.advertise<nav_msgs::Odometry>("/gps/odom", 10);
        // ros::Publisher pub_gps_path = nh.advertise<nav_msgs::Path>("/gps/path", 10);        

        // *************************************
        // read cloud list
        FILE *file;
        double base_time = ros::Time::now().toSec();
        double cloud_time;
        vector<double> cloud_time_list;
        {
            file = std::fopen((data_path + "cloud_0/timestamps.txt").c_str(), "r");
            if (!file)
            {
                printf("cannot find file: %stimes.txt\n", data_path.c_str());
                ROS_BREAK();
                return 0;
            }
            while (fscanf(file, "%lf", &cloud_time) != EOF)
            {
                cloud_time_list.push_back(cloud_time);
            }
            std::fclose(file);    
        }  

        // *************************************
        // read data
        size_t START_IDX = std::stoi(argv[4]);
        size_t END_IDX = std::stoi(argv[5]);
        printf("start idx: %d, end idx: %d, whole data size: %lu\n", START_IDX, END_IDX, cloud_time_list.size());
        for (size_t i = START_IDX; i < std::min(cloud_time_list.size(), END_IDX); i+=DELTA_IDX)
        {	
            if (ros::ok())
            {
                double cloud_time = cloud_time_list[i];
                // double cloud_time = ros::Time::now().toSec();
                printf("process data: %d\n", i);
                stringstream ss;
                ss << setfill('0') << setw(6) << i;

                // load cloud
                printf("size of finding cloud: ");
                std::vector<pcl::PointCloud<pcl::PointXYZI> > laser_cloud_list(4);
                for (size_t j = 0; j < 4; j++)
                {
                    stringstream ss_file;
                    ss_file << "cloud_" << j << "/data/" << ss.str() << ".pcd";
                    string cloud_path = data_path + ss_file.str();
                    if (pcl::io::loadPCDFile<pcl::PointXYZI>(cloud_path, laser_cloud_list[j]) == -1)
                    {
                        printf("Couldn't read file %s\n", cloud_path.c_str());
                        ROS_BREAK();
                        return 0;
                    }
                }
                pcl::PointCloud<pcl::PointXYZI> cloud_fused;
                {
                    pcl::PointCloud<pcl::PointXYZI> cloud;
                    pcl::transformPointCloud(laser_cloud_list[0], cloud, TBL[0].cast<float>());
                    for (auto &p : cloud.points) p.intensity = 0;
                    cloud_fused += cloud;
                }
                if (NUM_OF_LASER >= 2) 
                {
                    pcl::PointCloud<pcl::PointXYZI> cloud;
                    pcl::transformPointCloud(laser_cloud_list[1], cloud, TBL[1].cast<float>());
                    for (auto &p : cloud.points) p.intensity = 1;
                    cloud_fused += cloud;
                }
                if (NUM_OF_LASER >= 3) 
                {
                    pcl::PointCloud<pcl::PointXYZI> cloud;
                    pcl::transformPointCloud(laser_cloud_list[2], cloud, TBL[2].cast<float>());
                    for (auto &p : cloud.points) p.intensity = 2;
                    cloud_fused += cloud;
                }
                if (NUM_OF_LASER >= 4)
                {
                    pcl::PointCloud<pcl::PointXYZI> cloud;
                    pcl::transformPointCloud(laser_cloud_list[3], cloud, TBL[3].cast<float>());
                    for (auto &p : cloud.points) p.intensity = 3;
                    cloud_fused += cloud;
                }
                printf("size of fused cloud %d\n", cloud_fused.size());
                std_msgs::Header header;
                header.frame_id = "velo";
                header.stamp = ros::Time(cloud_time);
                common::publishCloud(cloud_fused_pub, header, cloud_fused);

                // load gps
                FILE *gps_file;
                string gps_file_path = data_path + "gps/data/" + ss.str() + ".txt";
                gps_file = std::fopen(gps_file_path.c_str() , "r");
                if(!gps_file)
                {
                    // printf("cannot find file: %s\n", gps_file_path.c_str());
                    // ROS_BREAK();
                    // return 0;          
                } else
                {
                    double lat, lon, alt;
                    double posx_accuracy, posy_accuracy, posz_accuracy;
                    int navstat, numsats;
                    fscanf(gps_file, "%d %d ", &navstat, &numsats);
                    fscanf(gps_file, "%lf %lf %lf ", &lat, &lon, &alt);
                    fscanf(gps_file, "%lf %lf %lf ", &posx_accuracy, &posy_accuracy, &posz_accuracy);
                    std::fclose(gps_file);

                    sensor_msgs::NavSatFix gps_msgs;
                    gps_msgs.header.frame_id = "gps";
                    gps_msgs.header.stamp = ros::Time(cloud_time);
                    gps_msgs.status.status = navstat;
                    gps_msgs.status.service = numsats;
                    gps_msgs.latitude = lat;
                    gps_msgs.longitude = lon;
                    gps_msgs.altitude = alt;
                    gps_msgs.position_covariance[0] = posx_accuracy;
                    gps_msgs.position_covariance[4] = posy_accuracy;
                    gps_msgs.position_covariance[8] = posz_accuracy;
                    pub_gps.publish(gps_msgs);
                }
                
                // load odom
                FILE *gt_odom_file;
                string gt_odom_file_path = data_path + "gt_odom/data/" + ss.str() + ".txt";
                gt_odom_file = std::fopen(gt_odom_file_path.c_str() , "r");
                if(!gt_odom_file)
                {
                    // printf("cannot find file: %s\n", gt_odom_file_path.c_str());
                    // ROS_BREAK();
                    // return 0;          
                } else
                {
                    double posx, posy, posz;
                    double orix, oriy, oriz, oriw;
                    fscanf(gt_odom_file, "%lf %lf %lf ", &posx, &posy, &posz);
                    fscanf(gt_odom_file, "%lf %lf %lf %lf ", &orix, &oriy, &oriz, &oriw);
                    std::fclose(gt_odom_file);

                    Eigen::Vector3d t_world_base(posx, posy, posz);
                    Eigen::Quaterniond q_world_base(oriw, orix, oriy, oriz);
                    Eigen::Quaterniond q_base_ref(1, 0, 0, 0);
                    Eigen::Vector3d t_base_ref(0, 0, 0);
                    Eigen::Quaterniond q_world_ref = q_world_base * q_base_ref;
                    Eigen::Vector3d t_world_ref = q_world_base * t_base_ref + t_world_base;
                    if (laser_gt_path.poses.size() == 0) 
                    {
                        q_world_ref_ini = q_world_ref;
                        t_world_ref_ini = t_world_ref;
                    }
                    Eigen::Quaterniond q_ref_ini_cur = q_world_ref_ini.inverse() * q_world_ref;
                    Eigen::Vector3d t_ref_ini_cur = q_world_ref_ini.inverse() * (t_world_ref - t_world_ref_ini);

                    nav_msgs::Odometry laser_gt_odom;
                    laser_gt_odom.header.frame_id = "/world";
                    laser_gt_odom.child_frame_id = "/gt";
                    laser_gt_odom.header.stamp = ros::Time(cloud_time);
                    laser_gt_odom.pose.pose.orientation.x = q_ref_ini_cur.x();
                    laser_gt_odom.pose.pose.orientation.y = q_ref_ini_cur.y();
                    laser_gt_odom.pose.pose.orientation.z = q_ref_ini_cur.z();
                    laser_gt_odom.pose.pose.orientation.w = q_ref_ini_cur.w();
                    laser_gt_odom.pose.pose.position.x = t_ref_ini_cur(0);
                    laser_gt_odom.pose.pose.position.y = t_ref_ini_cur(1);
                    laser_gt_odom.pose.pose.position.z = t_ref_ini_cur(2);
                    pub_laser_gt_odom.publish(laser_gt_odom);

                    static tf::TransformBroadcaster br;
                    tf::Transform transform;
                    tf::Quaternion q;
                    transform.setOrigin(tf::Vector3(laser_gt_odom.pose.pose.position.x,
                                                    laser_gt_odom.pose.pose.position.y,
                                                    laser_gt_odom.pose.pose.position.z));
                    q.setW(laser_gt_odom.pose.pose.orientation.w);
                    q.setX(laser_gt_odom.pose.pose.orientation.x);
                    q.setY(laser_gt_odom.pose.pose.orientation.y);
                    q.setZ(laser_gt_odom.pose.pose.orientation.z);
                    transform.setRotation(q);
                    br.sendTransform(tf::StampedTransform(transform, laser_gt_odom.header.stamp, 
                        laser_gt_odom.header.frame_id, laser_gt_odom.child_frame_id));                    

                    geometry_msgs::PoseStamped laser_gt_pose;
                    laser_gt_pose.header.frame_id = "/world";
                    laser_gt_pose.header.stamp = ros::Time(cloud_time);
                    laser_gt_pose.pose = laser_gt_odom.pose.pose;
                    laser_gt_path.header = laser_gt_pose.header;
                    laser_gt_path.poses.push_back(laser_gt_pose);
                    pub_laser_gt_path.publish(laser_gt_path);
                }

                ros::Rate loop_rate(10);
                if (b_pause)
                {
                    while (true)
                    {
                        ros::spinOnce();
                        if ((!b_pause) || (!ros::ok())) break;
                        loop_rate.sleep();
                    }
                } else
                {
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
            else
                break;
        }
    }

    return 0;
}
