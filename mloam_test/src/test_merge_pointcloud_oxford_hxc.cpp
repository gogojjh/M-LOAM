#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

// PCL
#include <pcl/point_cloud.h>			/* pcl::PointCloud */
#include <pcl/point_types.h>			/* pcl::PointXYZ */
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include <iostream>

using namespace std;

typedef sensor_msgs::PointCloud2 LidarMsgType;
typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType> LidarSyncPolicy;
typedef message_filters::Subscriber<LidarMsgType> LidarSubType;

ros::Publisher cloud_fused_pub;
nav_msgs::Path laser_gt_path;
ros::Publisher pub_laser_gt_path;

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

void process(const sensor_msgs::PointCloud2ConstPtr& pc2_left,
             const sensor_msgs::PointCloud2ConstPtr& pc2_right)
{
    pcl::PointCloud<pcl::PointXYZI> cloud, cloud_fused;
    std_msgs::Header header = pc2_left->header;
    {
        std::vector<double> calib{0, 0, 0, 1, 0, 0, 0};
        pcl::fromROSMsg(*pc2_left, cloud);
        Eigen::Matrix4d trans = getTransformMatrix(calib);
        pcl::transformPointCloud(cloud, cloud, trans.cast<float>());
        for (auto &p : cloud.points) p.intensity = 0;
        cloud_fused += cloud;
        // std::cout << cloud_fused.size() << std::endl;
    }
    {
        std::vector<double> calib{0.004008, 0.000008, 0.00099912, 0.9999, -0.0038006, -0.89758, -0.0031404};
        pcl::fromROSMsg(*pc2_right, cloud);
        Eigen::Matrix4d trans = getTransformMatrix(calib);
        pcl::transformPointCloud(cloud, cloud, trans.cast<float>());
        for (auto &p : cloud.points) p.intensity = 1;
        cloud_fused += cloud;
        // std::cout << cloud_fused.size() << std::endl;
    }
    if (cloud_fused.size())
    {
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(cloud, msg_cloud);
        msg_cloud.header = header;
        cloud_fused_pub.publish(msg_cloud);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_merge_pointcloud");
    ros::NodeHandle nh("~");
    cloud_fused_pub = nh.advertise<LidarMsgType>("/fused/velodyne_points", 5);

    LidarSubType *sub_left = new LidarSubType(nh, "/left/velodyne_points", 1);
    LidarSubType *sub_right = new LidarSubType(nh, "/right/velodyne_points", 1);
    message_filters::Synchronizer<LidarSyncPolicy> *lidar_synchronizer =
        new message_filters::Synchronizer<LidarSyncPolicy>(
            LidarSyncPolicy(10), *sub_left, *sub_right);
    lidar_synchronizer->registerCallback(boost::bind(&process, _1, _2));

    ros::Rate fps(100);
    while (ros::ok())
    {
        ros::spinOnce();
        fps.sleep();
    }
    return 0;
}

// ****************************************************************
// lego-loam: please insert in transformFusion.cpp the bottom of laserOdometryHandler()
std::ofstream fout("/tmp/stamped_legoloam_map_estimate.txt", std::ios::out);
for (size_t i = 0; i < laserAfterMappedPath.poses.size(); i++)
{
    geometry_msgs::PoseStamped &laser_pose = laserAfterMappedPath.poses[i];
    fout.precision(15);
    fout << laser_pose.header.stamp.toSec() << " ";
    fout.precision(8);
    fout << laser_pose.pose.position.x << " "
         << laser_pose.pose.position.y << " "
         << laser_pose.pose.position.z << " "
         << laser_pose.pose.orientation.x << " "
         << laser_pose.pose.orientation.y << " "
         << laser_pose.pose.orientation.z << " "
         << laser_pose.pose.orientation.w << std::endl;
}
fout.close();