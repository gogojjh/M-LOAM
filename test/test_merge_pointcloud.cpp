#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

// PCL
#include <pcl/point_cloud.h>			/* pcl::PointCloud */
#include <pcl/point_types.h>			/* pcl::PointXYZ */
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

typedef sensor_msgs::PointCloud2 LidarMsgType;
typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType> LidarSyncPolicy;
typedef message_filters::Subscriber<LidarMsgType> LidarSubType;

ros::Publisher cloud_fused_pub;

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

template <typename PointT>
static void publishCloud(const ros::Publisher& publisher,
                         const std_msgs::Header& header,
                         const typename pcl::PointCloud<PointT>& cloud)
{
    if (cloud.size())
    {
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(cloud, msg_cloud);
        msg_cloud.header = header;
        publisher.publish(msg_cloud);
    }
}

void process(const sensor_msgs::PointCloud2ConstPtr& pc2_left,
             const sensor_msgs::PointCloud2ConstPtr& pc2_right)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_left, cloud_right, cloud_fused;
    std_msgs::Header header = pc2_left->header;
    {
        std::vector<double> calib{0, 0, 0, 1, 0, 0, 0};
        pcl::fromROSMsg(*pc2_left, cloud_left);
        Eigen::Matrix4d trans = getTransformMatrix(calib);
        pcl::transformPointCloud(cloud_left, cloud_left, trans.cast<float>());
        for (auto &p : cloud_left.points) p.intensity = 0;
        cloud_fused += cloud_left;
    }
    {
        std::vector<double> calib{0.342, 0.0, 0.0, 0.940, 0, -0.477, 0.0};
        pcl::fromROSMsg(*pc2_right, cloud_right);
        Eigen::Matrix4d trans = getTransformMatrix(calib);
        pcl::transformPointCloud(cloud_right, cloud_right, trans.cast<float>());
        for (auto &p : cloud_right.points) p.intensity = 1;
        cloud_fused += cloud_right;
    }
    publishCloud(cloud_fused_pub, header, cloud_fused);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_merge_pointcloud");
    ros::NodeHandle nh("~");

    cloud_fused_pub = nh.advertise<LidarMsgType>("/fused/velodyne_points", 1);

    //register fusion callback function
    LidarSubType* sub_left = new LidarSubType(nh, "/left/velodyne_points", 1);
    LidarSubType* sub_right = new LidarSubType(nh, "/right/velodyne_points", 1);
    message_filters::Synchronizer<LidarSyncPolicy>* lidar_synchronizer =
        new message_filters::Synchronizer<LidarSyncPolicy>(LidarSyncPolicy(10), *sub_left, *sub_right);
    lidar_synchronizer->registerCallback(boost::bind(&process, _1, _2));

    ros::Rate fps(40);
    while (ros::ok())
	{
        ros::spinOnce();
        fps.sleep();
    }

    return 0;
}
