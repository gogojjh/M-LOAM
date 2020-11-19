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
#include <string>
#include <memory>
#include <mutex>
#include <queue>
#include <stack>
#include <thread>
#include <signal.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "common/color.hpp"
#include "common/publisher.hpp"
// #include "mloam_msgs/Keyframes.h"
// #include "eloam/keyframe.h"

#include "eloam/utility/event.hpp"
#include "eloam/parameters.hpp"
#include "eloam/event_processor.h"

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");
DEFINE_string(data_source, "bag", "the data source: bag or txt");

std::queue<dvs_msgs::Event> event_buf;
std::queue<sensor_msgs::Image::ConstPtr> image_buf;
std::queue<sensor_msgs::CameraInfo::ConstPtr> camera_info_buf;
std::queue<sensor_msgs::Imu::ConstPtr> imu_buf;
std::queue<geometry_msgs::PoseStamped::ConstPtr> pose_gt_buf;

std::vector<eloam::Event> event_cur;
sensor_msgs::CameraInfo camera_info;

double time_event_cur;

nav_msgs::Path gt_path;
Pose pose_world_ref_ini;

ros::Publisher pub_gt_path;

// loading parameter
int RESULT_SAVE;
std::string OUTPUT_FOLDER;
std::string ELOAM_PATH;
float SPATIO_TEMPORAL_WINDOW_SIZE;
float MEDIAN_BLUR_KERNEL_SIZE;
int VISUALIZE_POLARITY;

// initialize class
bool b_sensor_initialized = false;
bool b_camera_info = false;

EventProcessor eventprocessor;
std::mutex m_buf, m_process;

void eventCallback(const dvs_msgs::EventArray::ConstPtr &event_msg)
{
    m_buf.lock();
    if (!b_sensor_initialized) 
    {
        b_sensor_initialized = true;
    }
    for (const dvs_msgs::Event &e : event_msg->events)
    {
        if (e.x < 0 || e.x >= eventprocessor.sensor_size_.width 
         || e.y < 0 || e.y >= eventprocessor.sensor_size_.height)
            continue;
        event_buf.push(e);
    }
    m_buf.unlock();
}

void imageCallback(const sensor_msgs::Image::ConstPtr &image_msg)
{
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg)
{
    m_buf.lock();
    camera_info_buf.push(camera_info_msg);
    m_buf.unlock();
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
}

void poseGTCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_gt_msg)
{
    Pose pose_world_base(pose_gt_msg->pose);
    Pose pose_base_ref(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
    Pose pose_world_ref(pose_world_base * pose_base_ref);
    if (gt_path.poses.size() == 0) pose_world_ref_ini = pose_world_ref;
    Pose pose_ref_ini_cur(pose_world_ref_ini.inverse() * pose_world_ref);

    nav_msgs::Odometry gt_odom;
    gt_odom.header = pose_gt_msg->header;
    gt_odom.header.frame_id = "/world";
    gt_odom.child_frame_id = "/gt";
    gt_odom.pose.pose.orientation.x = pose_ref_ini_cur.q_.x();
    gt_odom.pose.pose.orientation.y = pose_ref_ini_cur.q_.y();
    gt_odom.pose.pose.orientation.z = pose_ref_ini_cur.q_.z();
    gt_odom.pose.pose.orientation.w = pose_ref_ini_cur.q_.w();
    gt_odom.pose.pose.position.x = pose_ref_ini_cur.t_(0);
    gt_odom.pose.pose.position.y = pose_ref_ini_cur.t_(1);
    gt_odom.pose.pose.position.z = pose_ref_ini_cur.t_(2);
    common::publishTF(gt_odom);

    geometry_msgs::PoseStamped gt_pose;
    gt_pose.header = pose_gt_msg->header;
    gt_pose.header.frame_id = "/world";
    gt_pose.pose = gt_odom.pose.pose;
    gt_path.header = gt_pose.header;
    gt_path.poses.push_back(gt_pose);
    pub_gt_path.publish(gt_path);
}

void process()
{
    while(1)
    {
		if (!ros::ok()) break;

        if (!camera_info_buf.empty())
        {
            m_buf.lock();
            if (!b_camera_info)
            {
                eventprocessor.setCameraInfo(camera_info_buf.front());
                b_camera_info = true;
            }
            camera_info_buf.pop();
            m_buf.unlock();
        }

        if (!event_buf.empty() && b_camera_info)
        {
            m_buf.lock();
            if (event_buf.size() >= SPATIO_TEMPORAL_WINDOW_SIZE)
            {
                while (event_cur.size() < SPATIO_TEMPORAL_WINDOW_SIZE)
                {
                    const dvs_msgs::Event &e = event_buf.front();
                    eloam::Event tmp_event(e.x, e.y, e.ts.toSec(), e.polarity);
                    event_cur.push_back(tmp_event);
                    event_buf.pop();
                }
            }          
            m_buf.unlock();
        }

        if (!image_buf.empty())
        {
            m_buf.lock();
            image_buf.pop();
            m_buf.unlock();
        }

        if (!imu_buf.empty())
        {
            m_buf.lock();
            imu_buf.pop();
            m_buf.unlock();
        }

        if (event_cur.size() != 0) eventprocessor.inputEvent(event_cur);
        event_cur.clear();

        // std::cout << common::RED << "keyframe size: " << eventprocessor.getKeyFrameSize() << common::RESET << std::endl << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void sigintHandler(int sig)
{
    printf("[eloam_node] press ctrl-c\n");
    // std::cout << common::YELLOW << "mapping drop frame: " << frame_drop_cnt << common::RESET << std::endl;
    if (RESULT_SAVE)
    {
        // m_process.lock();
        // eventprocessor.saveEventProcessor();
        // m_process.unlock();
    }
    ros::shutdown();
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("please intput: rosrun eloam eloam_node -help\n");
        return 1;
    }
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "eloam_node");
    ros::NodeHandle nh("~");

    // load and set basic parameters 
    RESULT_SAVE = FLAGS_result_save;
    OUTPUT_FOLDER = FLAGS_output_path;
    ELOAM_PATH = OUTPUT_FOLDER + "traj/eloam_odom_estimate.txt";
    printf("[eloam_node] save result (0/1): %d to %s\n", RESULT_SAVE, OUTPUT_FOLDER.c_str());
    printf("config_file: %s\n", FLAGS_config_file.c_str());

    cv::FileStorage fsSettings(FLAGS_config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    SPATIO_TEMPORAL_WINDOW_SIZE = fsSettings["spatio_temporal_window_size"];
    MEDIAN_BLUR_KERNEL_SIZE = fsSettings["median_blur_kernel_size"];
    VISUALIZE_POLARITY = fsSettings["visualize_polarity"];
    fsSettings.release();

    // initialize basis topics
    eventprocessor.registerPub(nh);
    // eventprocessor.setParameter();
    // eventprocessor.setPGOTread();

    if (FLAGS_data_source == "bag")
    {
        ros::Subscriber sub_event = nh.subscribe("/events", 0, eventCallback);
        ros::Subscriber sub_image = nh.subscribe("/image_raw", 1, imageCallback);
        ros::Subscriber sub_camera_info = nh.subscribe("/camera_info", 1, cameraInfoCallback);
        ros::Subscriber sub_imu = nh.subscribe("/imu", 1, imuCallback);
        ros::Subscriber sub_pose_gt = nh.subscribe("/optitrack/davis", 1, poseGTCallback);

        pub_gt_path = nh.advertise<nav_msgs::Path>("gt_path", 5);

        signal(SIGINT, sigintHandler);
        std::thread eloam_node_thread{process};
        ros::spin();
        eloam_node_thread.join();
    } 
    else if (FLAGS_data_source == "txt")
    {
        // time_event_init = ros::Time::now();
        // FILE *file = std::fopen((FLAGS_data_path + "/events.txt").c_str(), "r");
        // if (!file)
        // {
        //     printf("cannot find event file: %s\n", FLAGS_data_path.c_str());
        //     ROS_BREAK();
        //     return 0;
        // }
        // float t, x, y, p;
        // while (fscanf(file, "%f %f %f %f", &t, &x, &y, &p) != EOF)
        // {
            
        // }
        // std::fclose(file);
    }

    return 0;
}
