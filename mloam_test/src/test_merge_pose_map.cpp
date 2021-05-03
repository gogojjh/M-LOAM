/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of M-LOAM (https://ram-lab.com/file/jjiao/m-loam).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Jianhao JIAO (jiaojh1994@gmail.com)
 *******************************************************/
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <iomanip>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "g2oIO/PoseGraphIO.h"

using namespace std;

DEFINE_string(g2ofile, "pose_graph", "path to load the pose graph");
DEFINE_string(mapfile, "xx.pcd", "cloud of LiDAR map");
DEFINE_string(curbfold, "/", "fold path for curb files");
DEFINE_string(g2ofile_keynode, "keynode.txt", "path to load the number of key nodes");
DEFINE_double(angle, 0.0, "angle");

pcl::PCDWriter pcd_writer;
pcl::PCDReader pcdReader;

PoseGraphIO g2oDataLoader;

visualization_msgs::Marker Point2Marker(std::pair<float, float> p0, std::pair<float, float> p1, size_t markSize, std_msgs::Header header, char *tag)
{
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.id = markSize + 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    if (!strcmp(tag, "obs"))
    {
        marker.color.b = 1.0f;
    } 
    else if (!strcmp(tag, "stop"))
    {
        marker.color.r = 1.0f;
    }
    else
    {
        marker.color.g = 1.0f;
    }
    marker.color.a = 1.0;
    geometry_msgs::Point point0, point1;
    point0.x = p0.first;
    point0.y = p0.second;
    point1.x = p1.first;
    point1.y = p1.second;    
    marker.points.push_back(point0);
    marker.points.push_back(point1);
    return marker;
}

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        printf("please intput: rosrun mloam test_merge_pose_map -help\n");
        return 1;
    }
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "test_merge_pose_map");
    ros::NodeHandle nh("~");

    ros::Publisher pub_laserCloud = nh.advertise<sensor_msgs::PointCloud2>("/laserCloud", 1);
    ros::Publisher pub_laserOdom = nh.advertise<sensor_msgs::PointCloud2>("/laserOdom", 1);
    ros::Publisher pub_visObs = nh.advertise<visualization_msgs::MarkerArray>("/visObs_Markers", 1);
    ros::Publisher pub_visStop = nh.advertise<visualization_msgs::MarkerArray>("/visStop_Markers", 1);

    ros::Rate loop_rate(1);
    ros::Duration loop_duration(5.0);
    Eigen::AngleAxisd axis_lidar_baselink(FLAGS_angle / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_lidar_baselink(axis_lidar_baselink);
    // Eigen::Vector3d t_lidar_baselink(-1.525, 0.0, 0.0);
    Eigen::Vector3d t_lidar_baselink(0.0, 0.0, 0.0);
    while (ros::ok())
    {
        string g2ofile_KeyNodePath = FLAGS_g2ofile_keynode;
        ifstream keyNodeFile(g2ofile_KeyNodePath.c_str(), ios::in);
        set<int> keyNode_idx;
        while (!keyNodeFile.eof())
        {
            int idx;
            keyNodeFile >> idx;
            keyNode_idx.insert(idx);
        }
        keyNodeFile.close();

        string g2ofilePath = FLAGS_g2ofile;
        vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> g2oPose = 
            g2oDataLoader.getEigenPoseFromg2oFile(g2ofilePath);
        pcl::PointCloud<pcl::PointXYZRGB> laserPose;
        for (size_t i = 0; i < g2oPose.size(); i++)
        {
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            R << g2oPose[i](0, 0), g2oPose[i](0, 1), g2oPose[i](0, 2),
                 g2oPose[i](1, 0), g2oPose[i](1, 1), g2oPose[i](1, 2),
                 g2oPose[i](2, 0), g2oPose[i](2, 1), g2oPose[i](2, 2);
            t << g2oPose[i](0, 3), g2oPose[i](1, 3), g2oPose[i](2, 3);
            Eigen::Matrix3d R_w_baselink;
            Eigen::Vector3d t_w_baselink;
            R_w_baselink = q_lidar_baselink.toRotationMatrix() * R;
            t_w_baselink = q_lidar_baselink.toRotationMatrix() * t + t_lidar_baselink;
            pcl::PointXYZRGB posePoint;
            posePoint.x = static_cast<float>(t_w_baselink(0));
            posePoint.y = static_cast<float>(t_w_baselink(1));
            posePoint.z = static_cast<float>(t_w_baselink(2));
            if (keyNode_idx.find(static_cast<int>(i)) != keyNode_idx.end())
            {
                posePoint.r = 255;
                posePoint.g = 0;
                posePoint.b = 0;
            }
            else
            {
                posePoint.r = 255;
                posePoint.g = 255;
                posePoint.b = 255;                
            }
            laserPose.push_back(posePoint);
        }
        // cout << "pose size: " << laserPose.size() << endl;
        sensor_msgs::PointCloud2 laserPose_msg;
        pcl::toROSMsg(laserPose, laserPose_msg);
        laserPose_msg.header.stamp = ros::Time::now();
        laserPose_msg.header.frame_id = "/world";
        pub_laserOdom.publish(laserPose_msg);

        ////
        // pcl::PointCloud<pcl::PointXYZI> laserCloud;
        // pcdReader.read(string(FLAGS_mapfile), laserCloud);
        // for (auto &p : laserCloud.points)
        // {
        //     p.x = -p.x;
        //     p.y = p.y;
        //     p.z = -p.z;
        // }
        // // cout << "cloud size: " << laserCloud.size() << endl;
        // sensor_msgs::PointCloud2 laserCloud_msg;
        // pcl::toROSMsg(laserCloud, laserCloud_msg);
        // laserCloud_msg.header = laserPose_msg.header;
        // pub_laserCloud.publish(laserCloud_msg);
        // printf("pose size: %u, cloud size: %u\n", laserPose.size(), laserCloud.size());

        // ***************** read curb
        visualization_msgs::MarkerArray obsMarkers, stopSignMarkers;

        // observation area
        string obsPath = FLAGS_curbfold + "/observation_area.txt";
        ifstream obsFile(obsPath.c_str(), ios::in);
        std::vector<std::vector<std::pair<float, float> > > vAllObsPoints;
        size_t nObs;
        obsFile >> nObs;
        for (size_t i = 0; i < nObs; i++)
        {
            int nPoints;
            obsFile >> nPoints;
            std::vector<std::pair<float, float> > vObsPoints;
            for (size_t j = 0; j < nPoints; j++)
            {
                float x, y;
                obsFile >> x >> y;
                vObsPoints.emplace_back(x, y);
                size_t l, r;
                if (j == 0)
                    continue;
                else
                {
                    l = j - 1;
                    r = j;
                }
                obsMarkers.markers.push_back(Point2Marker(vObsPoints[l], vObsPoints[r], obsMarkers.markers.size(), laserPose_msg.header, "obs"));         
            }
            obsMarkers.markers.push_back(Point2Marker(vObsPoints[nPoints - 1], vObsPoints[0], obsMarkers.markers.size(), laserPose_msg.header, "obs"));                     
            vAllObsPoints.push_back(vObsPoints);
        }
        obsFile.close();
        pub_visObs.publish(obsMarkers);

        // observation area
        string stopSignPath = FLAGS_curbfold + "/stop_sign_area.txt";
        ifstream stopSignFile(stopSignPath.c_str(), ios::in);
        std::vector<std::vector<std::pair<float, float> > > vAllStopSignPoints;
        size_t nStopSign;
        stopSignFile >> nStopSign;
        for (size_t i = 0; i < nStopSign; i++)
        {
            int nPoints;
            stopSignFile >> nPoints;
            std::vector<std::pair<float, float> > vStopSignPoints;
            for (size_t j = 0; j < nPoints; j++)
            {
                float x, y;
                stopSignFile >> x >> y;
                vStopSignPoints.emplace_back(x, y);
                size_t l, r;
                if (j == 0)
                    continue;
                else
                {
                    l = j - 1;
                    r = j;
                }        
                stopSignMarkers.markers.push_back(Point2Marker(vStopSignPoints[l], vStopSignPoints[r], stopSignMarkers.markers.size(), laserPose_msg.header, "stop"));         
            }
            stopSignMarkers.markers.push_back(Point2Marker(vStopSignPoints[nPoints - 1], vStopSignPoints[0], stopSignMarkers.markers.size(), laserPose_msg.header, "stop"));         
            vAllStopSignPoints.push_back(vStopSignPoints);
        }
        stopSignFile.close();
        pub_visStop.publish(stopSignMarkers);

        printf("%u obs areas, %u stop sign areas\n", nObs, nStopSign);
        loop_duration.sleep();
    }
    return 0;
}