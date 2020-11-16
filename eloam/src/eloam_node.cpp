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
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

// #include "common/color.hpp"
// #include "mloam_msgs/Keyframes.h"
// #include "eloam/keyframe.h"

#include "eloam/parameters.hpp"
#include "eloam/event_frame.h"

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");

std::queue<dvs_msgs::EventArray::ConstPtr> event_buf;
std::queue<sensor_msgs::CameraInfo::ConstPtr> camera_info_buf;

// loading parameter
int RESULT_SAVE;
std::string OUTPUT_FOLDER;
std::string ELOAM_PATH;
float SPATIO_TEMPORAL_WINDOW_SIZE;

// initialize class
cv::Size sensor_size = cv::Size(0, 0);
bool b_sensor_initialized = false;
int event_frame_cnt = 0;

double time_event_last;

std::vector<dvs_msgs::EventArray> event_window;
sensor_msgs::CameraInfo camera_info;

EventFrame eventframe;
std::mutex m_buf, m_process;

void initSensor(int width, int height)
{
    sensor_size = cv::Size(width, height);
    b_sensor_initialized = true;
    printf("Sensor size: (%d x %d)\n", sensor_size.width, sensor_size.height);
}

void eventCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{
    if (!b_sensor_initialized) initSensor(msg->width, msg->height);
    event_buf.push(msg);
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    camera_info_buf.push(msg);
}

void process()
{
    while(1)
    {
		if (!ros::ok()) break;
        while (!event_buf.empty())
        {
            m_buf.lock();

            time_event_last = event_buf.front()->header.stamp.toSec();
            event_window.push_back(*event_buf.front());
            event_buf.pop();

            m_buf.unlock();

            // eventframe.inputEvent();
            event_frame_cnt++;            
            // std::cout << common::RED << "keyframe size: " << eventframe.getKeyFrameSize() << common::RESET << std::endl << std::endl;
            std::cout << "event frame cnt: " << event_frame_cnt << std::endl;
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void sigintHandler(int sig)
{
    printf("[eloam_node] press ctrl-c\n");
    // std::cout << common::YELLOW << "mapping drop frame: " << frame_drop_cnt << common::RESET << std::endl;
    if (RESULT_SAVE)
    {
        // m_process.lock();
        // eventframe.saveEventFrame();
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
    eventframe.registerPub(nh);

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
    std::cout << SPATIO_TEMPORAL_WINDOW_SIZE << std::endl;
    fsSettings.release();

    // eventframe.setParameter();
    // eventframe.setPGOTread();

    ros::Subscriber sub_event = nh.subscribe("/events", 0, eventCallback);
    ros::Subscriber sub_camera_info = nh.subscribe("/camera_info", 1, cameraInfoCallback);

    // // pub_laser_loop_keyframes_6d = nh.advertise<mloam_msgs::Keyframes>("/laser_loop_keyframes_6d", 5);
    // // pub_scan_context = nh.advertise<sensor_msgs::Image>("/input_scan_context", 5);

    signal(SIGINT, sigintHandler);
    std::thread eloam_node_thread{process};
    ros::spin();

    eloam_node_thread.join();
    return 0;
}
