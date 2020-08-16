#ifndef _TRANSFORM_HPP_
#define _TRANSFORM_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>          /* tf::TransformListener */
#include <tf_conversions/tf_eigen.h>        /* tf::transformTFToEigen */
#include <geometry_msgs/Point.h>            /* geometry_msgs::Point */

#include <string>
#include <Eigen/Core>

#include "types/object.hpp"               /* ObjectPtr */

namespace common
{
    namespace transform
    {
        static bool getVelodynePose(const tf::TransformListener& tf_buffer,
                                    const std::string& source_frame, const std::string& target_frame,
                                    const double& query_time, Eigen::Matrix4d* trans)
        {
            if (trans == nullptr) {
                ROS_ERROR("Failed to get trans, the trans ptr can not be NULL.");
                return false;
            }

            ros::Time query_stamp(query_time);
            tf::StampedTransform transform_stamped;
            try {
                tf_buffer.lookupTransform(target_frame, source_frame, query_stamp, transform_stamped);
            } catch (tf2::TransformException& ex) {
                //ROS_ERROR_STREAM("Exception: " << ex.what());
                ROS_WARN("Failed to query pose at %lf, use latest available pose instead.", query_time);
                tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), transform_stamped);
            }
            Eigen::Affine3d affine_3d;
            tf::transformTFToEigen(transform_stamped, affine_3d);
            *trans = affine_3d.matrix();

            ROS_INFO_STREAM("Get " << source_frame << " to " << target_frame << " trans: \n" << *trans);

            return true;
        }

        static void transformGroundBox(const Eigen::Matrix4d& trans,
                                       Eigen::Vector3d* position)
        {
            Eigen::Vector3d& center = *position;
            center = (trans * Eigen::Vector4d(center[0], center[1], center[2], 1)).head(3);
        }

        template <typename PointT>
        static void transformPoint(const Eigen::Matrix4d& trans,
                                   PointT* position)
        {
            Eigen::Vector3d center = (trans * Eigen::Vector4d((*position).x, (*position).y, (*position).z, 1)).head(3);
            (*position).x = center(0);
            (*position).y = center(1);
            (*position).z = center(2);
        }

        static void transformDirection(const Eigen::Matrix4d& trans,
                                       Eigen::Vector3d* position)
        {
            Eigen::Vector3d& center = *position;
            center = (trans * Eigen::Vector4d(center[0], center[1], center[2], 0)).head(3);
        }

        static void transformVelocity(const Eigen::Matrix4d& trans,
                                      Eigen::Vector3d* position)
        {
            transformDirection(trans, position);
        }

        template <typename PointT>
        static void transformPointCloud(const Eigen::Matrix4d& trans_mat,
                                        typename pcl::PointCloud<PointT>::Ptr cloud_in_out)
        {
            assert(cloud_in_out.get() != nullptr);

            for (size_t i = 0u; i < cloud_in_out->size(); ++i) {
                PointT& p = cloud_in_out->at(i);
                //ROS_WARN("Original: (%lf, %lf, %lf)", p.x, p.y, p.z);
                Eigen::Vector4d v(p.x, p.y, p.z, 1.);
                v = trans_mat * v;
                p.x = v(0);
                p.y = v(1);
                p.z = v(2);
                //ROS_INFO("Transformed: (%lf, %lf, %lf)", p.x, p.y, p.z);
            }
        }
        /**
         * @brief
         *  needed for PointICloud-converted tyoe, can't use Ptr
         *  common::geometry::transformPointCloud<PointD>
         * @tparam PointT
         * @param trans_mat
         * @param cloud_in_out
         */
        template <typename PointT>
        static void transformPointCloud(const Eigen::Matrix4d& trans_mat,
                                        typename pcl::PointCloud<PointT>& cloud_in_out)
        {
            for (size_t i = 0u; i < cloud_in_out.size(); ++i) {
                PointT& p = cloud_in_out.at(i);
                Eigen::Vector4d v(p.x, p.y, p.z, 1.);
                v = trans_mat * v;
                p.x = v(0);
                p.y = v(1);
                p.z = v(2);
            }
        }

        /**
         * @brief transform object with given pose
         * @params[IN] pose: pose using for coordinate transformation
         * @params[OUT] obj: object for transfromation
         * @return nothing
         */
        static void transformBuiltObject(const Eigen::Matrix4d& pose, ObjectPtr obj)
        {
            /*Eigen::Vector3d& dir = obj->direction;
            dir = (pose * Eigen::Vector4d(dir[0], dir[1], dir[2], 0)).head(3);*/
            transformDirection(pose, &(obj->direction));
            // transform center
            transformGroundBox(pose, &(obj->ground_center));
            /*Eigen::Vector3d& center = obj->ground_center;
            center = (pose * Eigen::Vector4d(center[0], center[1], center[2], 1)).head(3);*/
            // transform cloud & polygon
            transformPointCloud<PointI>(pose, obj->cloud);
            // PointDCloudPtr polygon(&((*obj)->polygon));
            transformPointCloud<PointD>(pose, obj->polygon);
        }

        static void transformBuiltObjects(const Eigen::Matrix4d& transform_to_mat,
                                          std::vector<ObjectPtr>* objects)
        {
            for (size_t i = 0u; i < (*objects).size(); ++i) {
                transformBuiltObject(transform_to_mat, (*objects)[i]);
            }
        }
    }
}

#endif /* _TRANSFORM_HPP_ */
