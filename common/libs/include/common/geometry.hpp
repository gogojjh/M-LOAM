#ifndef _GEOMETRY_HPP_
#define _GEOMETRY_HPP_

#include <Eigen/Core>
#include <pcl/point_cloud.h>

#include "./types/object.hpp"         /* ObjectPtr */

namespace common {

    namespace geometry {
        /**
         * @brief compute velocity's angle change between "v1" and "v2"
         * @note
         *  v1·v2 = |v1|*|v2|cos(theta)
         *  v1xv2 = |v1|*|v2|sin(theta)
         * @param v1
         * @param v2
         * @return
         */
        template <typename VectorT>
        static double computeTheta2dXyBetweenVectors(const VectorT& v1,
                                                     const VectorT& v2)
        {
            double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());
            double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());

            //cos(theta) = (v1·v2)/(|v1|*|v2|) = [v1(0)*v2(0)+v1(1)*v2(1)]/(|v1|*|v2|)
            double cos_theta = (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
            //limit theta to [0, PI] or [-PI, 0]
            if (cos_theta > 1) {
                cos_theta = 1;
            }
            if (cos_theta < -1) {
                cos_theta = -1;
            }

            //sin(theta) = (v1xv2)/(|v1|*|v2|) = [v1(0)*v2(1)-v1(1)*v2(0)]/(|v1|*|v2|)
            double sin_theta = (v1(0) * v2(1) - v1(1) * v2(0)) / (v1_len * v2_len);
            //return [0, PI]
            double theta = acos(cos_theta);
            if (sin_theta < 0) {
                theta = -theta;
            }

            return theta;
        }

        /**
         * @breif compute velocity's angle cos value between "v1" and "v2"
         * @param v1
         * @param v2
         * @return
         */
        static double computeCosTheta2dXyBetweenVectors(const Eigen::Vector3f& v1,
                                                        const Eigen::Vector3f& v2)
        {
            double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());
            double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());
            double cos_theta = (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
            return cos_theta;
        }

        /**
         * @breif compute 3D point's Sphere/Cylinder distance and corresponding norm
         * @tparam PointT
         * @param point
         * @return
         */
        template <typename PointT>
        static float calcSphereDistNorm(PointT point)
        {
            return pow(point.x, 2.0) + pow(point.y, 2.0) + pow(point.z, 2.0);
        }

        template <typename PointT>
        static float calcSphereDist(PointT point)
        {
            return sqrt(calcSphereDistNorm<PointT>(point));
        }

        template <typename PointT>
        static float calcCylinderDistNorm(PointT point)
        {
            return pow(point.x, 2.0) + pow(point.y, 2.0);
        }

        template <typename PointT>
        static float calcCylinderDist(PointT point)
        {
            return sqrt(calcCylinderDistNorm<PointT>(point));
        }

        // 计算点云重心
        template <typename PointT>
        static Eigen::Vector3d getCloudBarycenter(typename pcl::PointCloud<PointT>::ConstPtr cloud)
        {
            int num_points = cloud->points.size();
            Eigen::Vector3d barycenter(0, 0, 0);

            for (int i = 0; i < num_points; i++) {
                const PointT& pt = cloud->points[i];
                barycenter[0] += pt.x;
                barycenter[1] += pt.y;
                barycenter[2] += pt.z;
            }

            if (num_points > 0) {
                barycenter[0] /= num_points;
                barycenter[1] /= num_points;
                barycenter[2] /= num_points;
            }
            return barycenter;
        }

        /**
         * @brief calculate yaw given direction vector, suppose coordinate vector: (1.0, 0.0, 0.0)
         * @tparam VectorT
         * @param dir
         * @return yaw in rad
         */
        template <typename VectorT>
        static double calcYaw4DirectionVector(const VectorT& dir)
        {
            const Eigen::Vector3d coord_dir(1.0, 0.0, 0.0);
            return computeTheta2dXyBetweenVectors<Eigen::Vector3d>(coord_dir, dir);
        }
    }
}

#endif  /* _GEOMETRY_HPP_ */