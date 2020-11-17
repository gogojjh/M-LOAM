#ifndef _PUBLISHER_HPP_
#define _PUBLISHER_HPP_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>            /* std_msgs, sensor_msgs */
#include <std_msgs/ColorRGBA.h>                 /* std_msgs::ColorRGBA */
#include <visualization_msgs/MarkerArray.h>     /* visualization_msgs::MarkerArray */
#include <tf/transform_broadcaster.h>

#include <pcl/common/common.h>                  /* pcl::getMinMax3D */
#include <pcl_conversions/pcl_conversions.h>

#include "types/type.h"
#include "transform.hpp"                      /* common::transform::transformPointCloud */
#include "types/object.hpp"                   /* ObjectPtr */
#include "geometry.hpp"                       /* common::geometry::calcYaw4DirectionVector */

namespace common {

    static void publishTF(const nav_msgs::Odometry &odom)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,
                                        odom.pose.pose.position.y,
                                        odom.pose.pose.position.z));
        q.setW(odom.pose.pose.orientation.w);
        q.setX(odom.pose.pose.orientation.x);
        q.setY(odom.pose.pose.orientation.y);
        q.setZ(odom.pose.pose.orientation.z);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, odom.header.frame_id, odom.child_frame_id));
    }

    template <typename PointT>
    static void publishCloud(const ros::Publisher& publisher,
                             const std_msgs::Header& header,
                             const typename pcl::PointCloud<PointT>& cloud)
    {
        if (cloud.size()) {
            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(cloud, msg_cloud);
            msg_cloud.header = header;
            publisher.publish(msg_cloud);
        }
    }

    /**
     * @brief publish clustering objects' in one point cloud
     * @param publisher
     * @param header
     * @param cloud_clusters
     * @param trans
     */
    template <typename PointT>
    static void publishClustersCloud(const ros::Publisher& publisher,
                                     const std_msgs::Header& header,
                                     const std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters_array,
                                     const Eigen::Matrix4d& trans = Eigen::Matrix4d::Zero())
    {
        if (clusters_array.size() <= 0) {
            ROS_WARN("Publish empty clusters cloud.");
            //publish empty cloud
            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*(new PointICloud), msg_cloud);
            msg_cloud.header = header;
            publisher.publish(msg_cloud);
            return;
        }
        else {
            ROS_INFO_STREAM("Publishing " << clusters_array.size() << " clusters in one cloud.");
        }

        PointICloudPtr cloud(new PointICloud);
        // different clusters with different intensity
        float step_i = 255.0f / clusters_array.size();
        for (size_t cluster_idx = 0u; cluster_idx < clusters_array.size(); ++cluster_idx) {
            if (clusters_array[cluster_idx]->points.size() <= 0) {
                ROS_WARN_STREAM("An empty cluster #" << cluster_idx << ".");
                continue;
            }
            for (size_t idx = 0u; idx < clusters_array[cluster_idx]->points.size(); ++idx) {
                PointI point;
                point.x = clusters_array[cluster_idx]->points[idx].x;
                point.y = clusters_array[cluster_idx]->points[idx].y;
                point.z = clusters_array[cluster_idx]->points[idx].z;
                point.intensity = cluster_idx * step_i;
                cloud->points.push_back(point);
            }
        }
        if ((trans.array() != 0.0).any()) {
            common::transform::transformPointCloud<PointI>(trans, cloud);
        }

        if (cloud->size()) {
            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*cloud, msg_cloud);
            msg_cloud.header = header;
            publisher.publish(msg_cloud);
        }
    }
    static void publishClustersCloud(const ros::Publisher& publisher,
                                     const std_msgs::Header& header,
                                     const std::vector<ObjectPtr>& objects_array,
                                     const Eigen::Matrix4d& trans = Eigen::Matrix4d::Zero())
    {
        if (objects_array.size() <= 0) {
            ROS_WARN("Publish empty clusters cloud.");
            //publish empty cloud
            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*(new PointICloud), msg_cloud);
            msg_cloud.header = header;
            publisher.publish(msg_cloud);
            return;
        }
        else {
            ROS_INFO_STREAM("Publishing " << objects_array.size() << " clusters in one cloud.");
        }

        PointICloudPtr cloud(new PointICloud);
        // different clusters with different intensity
        float step_i = 255.0f / objects_array.size();
        for (size_t cluster_idx = 0u; cluster_idx < objects_array.size(); ++cluster_idx) {
            PointICloudConstPtr cloud_tmp(objects_array[cluster_idx]->cloud);
            if (cloud_tmp->points.size() <= 0) {
                ROS_WARN_STREAM("An empty cluster #" << cluster_idx << ".");
                continue;
            }
            for (size_t idx = 0u; idx < cloud_tmp->points.size(); ++idx) {
                PointI point;
                point.x = cloud_tmp->points[idx].x;
                point.y = cloud_tmp->points[idx].y;
                point.z = cloud_tmp->points[idx].z;
                point.intensity = cluster_idx * step_i;
                cloud->points.push_back(point);
            }
        }
        if ((trans.array() != 0.0).any()) {
            common::transform::transformPointCloud<PointI>(trans, cloud);
        }

        if (cloud->size()) {
            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*cloud, msg_cloud);
            msg_cloud.header = header;
            publisher.publish(msg_cloud);
        }
    }

    /**
     * @brief publish Objects' 3D OBB and velocity arrow
     * @param publisher
     * @param header
     * @param color
     * @param objects
     * @param trans
     */
    static void publishObjectsMarkers(const ros::Publisher& publisher,
                                      const std_msgs::Header& header,
                                      const std_msgs::ColorRGBA& color,
                                      const std::vector<ObjectPtr>& objects_array,
                                      const Eigen::Matrix4d& trans = Eigen::Matrix4d::Zero())
    {
        //clear all markers before
        visualization_msgs::MarkerArray empty_markers;
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "objects";
        clear_marker.id = 0;
        clear_marker.action = clear_marker.DELETEALL;
        clear_marker.lifetime = ros::Duration();
        empty_markers.markers.push_back(clear_marker);
        publisher.publish(empty_markers);

        if (objects_array.size() <= 0)
        {
            ROS_WARN("Publish empty object marker.");
            return;
        }
        else
        {
            ROS_INFO("Publishing %lu objects markers.", objects_array.size());
        }

        visualization_msgs::MarkerArray object_markers;
        for (size_t obj = 0u; obj < objects_array.size(); ++obj)
        {
            /*
             * @note Apollo's Object Coordinate
             *          |x
             *      C   |   D-----------
             *          |              |
             *  y---------------     length
             *          |              |
             *      B   |   A-----------
             */
            Eigen::Vector3d center = objects_array[obj]->ground_center;
            Eigen::Vector3d dir = objects_array[obj]->direction;
            if ((trans.array() != 0.0).any()) {
                common::transform::transformGroundBox(trans, &center);
                common::transform::transformDirection(trans, &dir);
            }
            //object size
            const double& length = objects_array[obj]->length;
            const double& width = objects_array[obj]->width;
            const double& height = objects_array[obj]->height;
            const double yaw = common::geometry::calcYaw4DirectionVector(dir);
            Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
            Eigen::Vector3d odir(-ldir[1], ldir[0], 0);
            Eigen::Vector3d bottom_quad[8];
            double half_l = length / 2;
            double half_w = width / 2;
            double h = height;
            //A(-half_l, -half_w)
            bottom_quad[0] = center + ldir * -half_l + odir * -half_w;
            //B(-half_l, half_w)
            bottom_quad[1] = center + ldir * -half_l + odir * half_w;
            //C(half_l, half_w)
            bottom_quad[2] = center + ldir * half_l + odir * half_w;
            //D(half_l, -half_w)
            bottom_quad[3] = center + ldir * half_l + odir * -half_w;
            //top 4 vertices
            bottom_quad[4] = bottom_quad[0]; bottom_quad[4](2) += h;
            bottom_quad[5] = bottom_quad[1]; bottom_quad[5](2) += h;
            bottom_quad[6] = bottom_quad[2]; bottom_quad[6](2) += h;
            bottom_quad[7] = bottom_quad[3]; bottom_quad[7](2) += h;

            Eigen::MatrixXf OBB(8, 3);
            OBB <<  bottom_quad[0](0),bottom_quad[0](1),bottom_quad[0](2),
                    bottom_quad[1](0),bottom_quad[1](1),bottom_quad[1](2),
                    bottom_quad[2](0),bottom_quad[2](1),bottom_quad[2](2),
                    bottom_quad[3](0),bottom_quad[3](1),bottom_quad[3](2),
                    bottom_quad[4](0),bottom_quad[4](1),bottom_quad[4](2),
                    bottom_quad[5](0),bottom_quad[5](1),bottom_quad[5](2),
                    bottom_quad[6](0),bottom_quad[6](1),bottom_quad[6](2),
                    bottom_quad[7](0),bottom_quad[7](1),bottom_quad[7](2);

            visualization_msgs::Marker box, dir_arrow;
            box.header = dir_arrow.header = header;
            box.ns = dir_arrow.ns = "objects";
            box.id = obj; dir_arrow.id = obj + objects_array.size();
            box.type = visualization_msgs::Marker::LINE_LIST;
            dir_arrow.type = visualization_msgs::Marker::ARROW;
            geometry_msgs::Point p[24];
            //Ground side
            //A->B
            p[0].x = OBB(0,0); p[0].y = OBB(0,1); p[0].z = OBB(0,2);
            p[1].x = OBB(1,0); p[1].y = OBB(1,1); p[1].z = OBB(1,2);
            //B->C
            p[2].x = OBB(1,0); p[2].y = OBB(1,1); p[2].z = OBB(1,2);
            p[3].x = OBB(2,0); p[3].y = OBB(2,1); p[3].z = OBB(2,2);
            //C->D
            p[4].x = OBB(2,0); p[4].y = OBB(2,1); p[4].z = OBB(2,2);
            p[5].x = OBB(3,0); p[5].y = OBB(3,1); p[5].z = OBB(3,2);
            //D->A
            p[6].x = OBB(3,0); p[6].y = OBB(3,1); p[6].z = OBB(3,2);
            p[7].x = OBB(0,0); p[7].y = OBB(0,1); p[7].z = OBB(0,2);

            //Top side
            //E->F
            p[8].x = OBB(4,0); p[8].y = OBB(4,1); p[8].z = OBB(4,2);
            p[9].x = OBB(5,0); p[9].y = OBB(5,1); p[9].z = OBB(5,2);
            //F->G
            p[10].x= OBB(5,0); p[10].y= OBB(5,1); p[10].z= OBB(5,2);
            p[11].x= OBB(6,0); p[11].y= OBB(6,1); p[11].z= OBB(6,2);
            //G->H
            p[12].x= OBB(6,0); p[12].y= OBB(6,1); p[12].z= OBB(6,2);
            p[13].x= OBB(7,0); p[13].y= OBB(7,1); p[13].z= OBB(7,2);
            //H->E
            p[14].x= OBB(7,0); p[14].y= OBB(7,1); p[14].z= OBB(7,2);
            p[15].x= OBB(4,0); p[15].y= OBB(4,1); p[15].z= OBB(4,2);

            //Around side
            //A->E
            p[16].x= OBB(0,0); p[16].y= OBB(0,1); p[16].z= OBB(0,2);
            p[17].x= OBB(4,0); p[17].y= OBB(4,1); p[17].z= OBB(4,2);
            //B->F
            p[18].x= OBB(1,0); p[18].y= OBB(1,1); p[18].z= OBB(1,2);
            p[19].x= OBB(5,0); p[19].y= OBB(5,1); p[19].z= OBB(5,2);
            //C->G
            p[20].x= OBB(2,0); p[20].y= OBB(2,1); p[20].z= OBB(2,2);
            p[21].x= OBB(6,0); p[21].y= OBB(6,1); p[21].z= OBB(6,2);
            //D->H
            p[22].x= OBB(3,0); p[22].y= OBB(3,1); p[22].z= OBB(3,2);
            p[23].x= OBB(7,0); p[23].y= OBB(7,1); p[23].z= OBB(7,2);

            for (size_t pi = 0u; pi < 24; ++pi) {
                box.points.push_back(p[pi]);
            }
            box.scale.x = 0.1;
            box.color = color;
            object_markers.markers.push_back(box);

            //direction
            geometry_msgs::Point start_point, end_point;
            Eigen::Vector3d end = center + dir * (length);
            start_point.x = center[0];  start_point.y = center[1];  start_point.z = center[2];
            end_point.x = end[0];       end_point.y = end[1];       end_point.z = end[2];
            dir_arrow.points.push_back(start_point);
            dir_arrow.points.push_back(end_point);
            dir_arrow.scale.x = 0.1;
            dir_arrow.scale.y = 0.2;
            dir_arrow.scale.z = length * 0.1;
            dir_arrow.color.a = 1.0;
            dir_arrow.color.r = 1.0;
            dir_arrow.color.g = 0.0;
            dir_arrow.color.b = 0.0;

            object_markers.markers.push_back(dir_arrow);
        }
        publisher.publish(object_markers);
    }

    /**
     * @brief publish Clusters' Min-Max Size box
     * @param publisher
     * @param header
     * @param color
     * @param objects
     * @param trans
     */
    static void publishMinMaxMarkers(const ros::Publisher& publisher,
                                     const std_msgs::Header& header,
                                     const std_msgs::ColorRGBA& color,
                                     const std::vector<PointICloudPtr>& clusters_array,
                                     const Eigen::Matrix4d& trans = Eigen::Matrix4d::Zero())
    {
        //clear all markers before
        visualization_msgs::MarkerArray empty_markers;
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "clusters";
        clear_marker.id = 0;
        clear_marker.action = clear_marker.DELETEALL;
        clear_marker.lifetime = ros::Duration();
        empty_markers.markers.push_back(clear_marker);
        publisher.publish(empty_markers);

        if (clusters_array.size() <= 0) {
            ROS_WARN("Publish empty cluster marker.");
            return;
        }
        else {
            ROS_INFO("Publishing %lu clusters markers.", clusters_array.size());
        }

        visualization_msgs::MarkerArray cluster_markers;
        for (size_t seg = 0u; seg < clusters_array.size(); ++seg) {
            PointI pt_min, pt_max;
            pcl::getMinMax3D(*clusters_array[seg], pt_min, pt_max);
            //transform into local coordinate
            if ((trans.array() != 0.0).any()) {
                common::transform::transformPoint<PointI>(trans, &pt_min);
                common::transform::transformPoint<PointI>(trans, &pt_max);
            }

            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "clusters";
            marker.id = seg;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            geometry_msgs::Point p[24];
            p[0].x  = pt_max.x; p[0].y  = pt_max.y; p[0].z  = pt_max.z;
            p[1].x  = pt_min.x; p[1].y  = pt_max.y; p[1].z  = pt_max.z;
            p[2].x  = pt_max.x; p[2].y  = pt_max.y; p[2].z  = pt_max.z;
            p[3].x  = pt_max.x; p[3].y  = pt_min.y; p[3].z  = pt_max.z;
            p[4].x  = pt_max.x; p[4].y  = pt_max.y; p[4].z  = pt_max.z;
            p[5].x  = pt_max.x; p[5].y  = pt_max.y; p[5].z  = pt_min.z;
            p[6].x  = pt_min.x; p[6].y  = pt_min.y; p[6].z  = pt_min.z;
            p[7].x  = pt_max.x; p[7].y  = pt_min.y; p[7].z  = pt_min.z;
            p[8].x  = pt_min.x; p[8].y  = pt_min.y; p[8].z  = pt_min.z;
            p[9].x  = pt_min.x; p[9].y  = pt_max.y; p[9].z  = pt_min.z;
            p[10].x = pt_min.x; p[10].y = pt_min.y; p[10].z = pt_min.z;
            p[11].x = pt_min.x; p[11].y = pt_min.y; p[11].z = pt_max.z;
            p[12].x = pt_min.x; p[12].y = pt_max.y; p[12].z = pt_max.z;
            p[13].x = pt_min.x; p[13].y = pt_max.y; p[13].z = pt_min.z;
            p[14].x = pt_min.x; p[14].y = pt_max.y; p[14].z = pt_max.z;
            p[15].x = pt_min.x; p[15].y = pt_min.y; p[15].z = pt_max.z;
            p[16].x = pt_max.x; p[16].y = pt_min.y; p[16].z = pt_max.z;
            p[17].x = pt_max.x; p[17].y = pt_min.y; p[17].z = pt_min.z;
            p[18].x = pt_max.x; p[18].y = pt_min.y; p[18].z = pt_max.z;
            p[19].x = pt_min.x; p[19].y = pt_min.y; p[19].z = pt_max.z;
            p[20].x = pt_max.x; p[20].y = pt_max.y; p[20].z = pt_min.z;
            p[21].x = pt_min.x; p[21].y = pt_max.y; p[21].z = pt_min.z;
            p[22].x = pt_max.x; p[22].y = pt_max.y; p[22].z = pt_min.z;
            p[23].x = pt_max.x; p[23].y = pt_min.y; p[23].z = pt_min.z;
            for (int i = 0; i < 24; i++) {
                marker.points.push_back(p[i]);
            }

            marker.scale.x = 0.1;
            marker.color = color;

            cluster_markers.markers.push_back(marker);
        }
        publisher.publish(cluster_markers);
    }


//    /**
//     * @brief publish self-defined clouds array
//     * @tparam CloudT
//     * @param publisher
//     * @param header
//     * @param cloud_segments
//     */
//    template <typename CloudT>
//    static void publishSegmentsArray(const ros::Publisher& publisher,
//                                     const std_msgs::Header& header,
//                                     const std::vector<CloudT>& cloud_segments)
//    {
//        if (cloud_segments.size() <= 0) {
//            ROS_WARN("Publish empty result segments.");
//            //publish empty cloud array
//            lidartld_msgs::PointCloud2Array segments_msg;
//            segments_msg.header = header;
//            publisher.publish(segments_msg);
//
//            return;
//        }
//        else {
//            ROS_INFO_STREAM("Publishing " << cloud_segments.size() << " segments.");
//        }
//
//        lidartld_msgs::PointCloud2Array segments_msg;
//        std::vector<sensor_msgs::PointCloud2> clouds;
//
//        for (size_t idx = 0u; idx < cloud_segments.size(); ++idx) {
//            if (cloud_segments[idx]->points.size() <= 0) {
//                ROS_WARN_STREAM("An empty Segment #" << idx << ".");
//                continue;
//            }
//            sensor_msgs::PointCloud2 cloud;
//            pcl::toROSMsg(*cloud_segments[idx], cloud);
//            clouds.push_back(cloud);
//        }
//
//        if (clouds.size()) {
//            segments_msg.header = header;
//            segments_msg.clouds = clouds;
//
//            publisher.publish(segments_msg);
//        }
//    }
//void SegBasedDetector::publishResultSegments(const std_msgs::Header& header,
//                                       const std::vector<ObjectPtr>& obj_clusters)
//{
//    if (obj_clusters.size() <= 0) {
//        ROS_WARN("Publish empty result segments.");
//        return;
//    }
//    else {
//        ROS_INFO_STREAM("Publishing " << obj_clusters.size() << " segments.");
//    }
//
//    lidartld_msgs::PointCloud2Array clouds_msg;
//    std::vector<sensor_msgs::PointCloud2> clouds;
//
//    for (size_t obj_idx = 0; obj_idx < obj_clusters.size(); obj_idx++) {
//        if (obj_clusters[obj_idx]->cloud->points.size() <= 0) {
//            ROS_WARN_STREAM("An empty Segment #" << obj_idx << ".");
//            continue;
//        }
//        sensor_msgs::PointCloud2 cloud;
//        pcl::toROSMsg(*obj_clusters[obj_idx]->cloud, cloud);
//        clouds.push_back(cloud);
//    }
//
//    if (clouds.size()) {
//        clouds_msg.header.frame_id = frame_id_;
//        clouds_msg.header.stamp = header.stamp;
//
//        clouds_msg.header = header;
//        clouds_msg.clouds = clouds;
//
//        result_segments_pub_.publish(clouds_msg);
//    }
//}
}

#endif /* _PUBLISHER_HPP_ */
