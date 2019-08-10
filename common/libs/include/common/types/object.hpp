/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef _OBJECT_HPP_
#define _OBJECT_HPP_

#include "type.h"
#include "feature.hpp"
#include <pcl/io/io.h>              /* pcl::copyPointCloud */
#include <Eigen/Core>

using Eigen::Vector3d;

namespace common
{
    typedef PointCloud PolygonType;
    typedef PointDCloud PolygonDType;

    struct alignas(16) Object {
        Object()
        {
            ground_center = Vector3d::Zero();
            velocity = Vector3d::Zero();
            direction = Vector3d(1, 0, 0);

            cloud.reset(new PointICloud);

            //type = UNKNOWN;
            //type_probs.resize(MAX_OBJECT_TYPE, 0);

            /*
             *
             * | 0.01  0    0   |
             * |  0   0.01  0   |
             * |  0    0   0.01 |
             */
            position_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
            velocity_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
        }

        /**
         * @brief deep copy of Object
         * @param rhs
         */
        void clone(const Object& rhs)
        {
            *this = rhs;
            //TODO pay attention to point when deep copy
            this->cloud.reset(new PointICloud);
            pcl::copyPointCloud<PointI, PointI>(*(rhs.cloud), *cloud);
        }

        void setId(IdType id)
        {
            this->id = id;
        }

        /*std::string ToString() const
        {
            // StrCat supports 9 arguments at most.
            return StrCat(StrCat("Object[id: ", id,
                                 ", "
                                         "track_id: ",
                                 track_id,
                                 ", "
                                         "cloud_size: ",
                                 cloud->size(),
                                 ", "
                                         "direction: ",
                                 Print(direction.transpose()), ", "),
                          StrCat("center: ", Print(center.transpose()),
                                 ", "
                                         "velocity: ",
                                 Print(velocity.transpose()),
                                 ", "
                                         "width: ",
                                 width,
                                 ", "
                                         "length: ",
                                 length, ", "),
                          StrCat("height: ", height,
                                 ", "
                                         "polygon_size: ",
                                 polygon.size(),
                                 ", "
                                         "type: ",
                                 type,
                                 ", "
                                         "is_background: ",
                                 is_background, "]"));
        }*/

        /*void Serialize(PerceptionObstacle *pb_obj) const
        {
            CHECK(pb_obj != NULL);
            pb_obj->set_id(track_id);
            pb_obj->set_theta(theta);

            Point *obj_center = pb_obj->mutable_position();
            obj_center->set_x(center(0));
            obj_center->set_y(center(1));
            obj_center->set_z(center(2));

            Point *obj_velocity = pb_obj->mutable_velocity();
            obj_velocity->set_x(velocity(0));
            obj_velocity->set_y(velocity(1));
            obj_velocity->set_z(velocity(2));

            pb_obj->set_length(length);
            pb_obj->set_width(width);
            pb_obj->set_height(height);

            for (auto point : polygon.points) {
                Point *p = pb_obj->add_polygon_point();
                p->set_x(point.x);
                p->set_y(point.y);
                p->set_z(point.z);
            }

            if (FLAGS_is_serialize_point_cloud) {
                for (auto point : cloud->points) {
                    pb_obj->add_point_cloud(point.x);
                    pb_obj->add_point_cloud(point.y);
                    pb_obj->add_point_cloud(point.z);
                }
            }

            pb_obj->set_tracking_time(tracking_time);
            pb_obj->set_type(static_cast<PerceptionObstacle::Type>(type));
            pb_obj->set_timestamp(latest_tracked_time);  // in seconds.
        }*/

        /*void Deserialize(const PerceptionObstacle& pb_obs)
        {
            track_id = pb_obs.id();
            theta = pb_obs.theta();

            center(0) = pb_obs.position().x();
            center(1) = pb_obs.position().y();
            center(2) = pb_obs.position().z();

            velocity(0) = pb_obs.velocity().x();
            velocity(1) = pb_obs.velocity().y();
            velocity(2) = pb_obs.velocity().z();

            length = pb_obs.length();
            width = pb_obs.width();
            height = pb_obs.height();

            polygon.clear();
            for (int idx = 0; idx < pb_obs.polygon_point_size(); ++idx) {
                const auto& p = pb_obs.polygon_point(idx);
                pcl_util::PointD point;
                point.x = p.x();
                point.y = p.y();
                point.z = p.z();
                polygon.push_back(point);
            }

            tracking_time = pb_obs.tracking_time();
            latest_tracked_time = pb_obs.timestamp();
            type = static_cast<ObjectType>(pb_obs.type());
        }*/

        /*std::string ToString() const
        {
            std::ostringstream oss;
            oss << "sensor_type: " << GetSensorType(sensor_type)
                << ", timestamp:" << GLOG_TIMESTAMP(timestamp)
                << ", sensor2world_pose:\n";
            oss << sensor2world_pose << "\n, objects: " << objects.size() << " < ";
            for (auto obj : objects) {
                oss << "\n" << obj->ToString();
            }
            oss << " >]";
            return oss.str();
        }*/

        //---------------------- basic information
        // object id per frame
        IdType id = -1;
        // point cloud of the object
        PointICloudPtr cloud;
        // convex hull of the object
        PolygonDType polygon;
        /*
         * @note Apollo's Object Coordinate
         *          |x
         *      C   |   D-----------
         *          |              |
         *  y---------------     length
         *          |              |
         *      B   |   A-----------
         */
        // oriented boundingbox information: main direction vector(x, y, 0)
        Eigen::Vector3d direction;
        // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
        double yaw_rad = 0.0;
        // ground center of the object (cx, cy, z_min)
        Eigen::Vector3d ground_center;
        // size of the oriented bbox, length is the size in the main direction
        double length = 0.0;
        double width = 0.0;
        double height = 0.0;

        //---------------------- classification information
        ObjectType type;
        // foreground score/probability
        float score = 0.0;
        // fg/bg flag
        bool is_background = false;
        // Object classification type.
        //ObjectType type;
        // Probability of each type, used for track type.
        //std::vector<float> type_probs;

        //---------------------- tracking information
        // shape feature used for tracker-observation match
        Feature shape_features;
        ///@note one tracker maintaina tracked trajectory
        IdType tracker_id = 0;
        // tracking state
        // stable anchor_point during time, e.g., barycenter
        Eigen::Vector3d anchor_point;
        Eigen::Vector3d velocity;
        // age of the tracked object
        double tracking_time = 0.0;
        double latest_tracked_time = 0.0;
        // noise covariance matrix for uncertainty of position and velocity
        Eigen::Matrix3d position_uncertainty;
        Eigen::Matrix3d velocity_uncertainty;
    };

    typedef std::shared_ptr<Object> ObjectPtr;
    typedef std::shared_ptr<const Object> ObjectConstPtr;
}

#endif /* _OBJECT_HPP_ */
