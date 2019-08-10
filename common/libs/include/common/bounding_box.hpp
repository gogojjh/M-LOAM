#ifndef _BOUNDING_BOX_HPP_
#define _BOUNDING_BOX_HPP_

#include <cmath>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/numeric/ublas/matrix.hpp>

#include <boost/geometry/geometries/adapted/c_array.hpp>
BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)

#include <Eigen/Core>

#include "./types/object.hpp"
#include "./common.hpp"

namespace common {

    namespace bbox {
        /*
         *                           |x
         *(x_max,y_max) ---width-----|
         *      |                    |
         *      |                  length
         *      |                    |
         *  y<-----------------(x_min,y_min)
         */
        typedef struct {
            double x_max;      // left-top corner
            double y_max;
            double x_min;      // right-bottom corner
            double y_min;
        } BoundingBox;

        //Orientation Bounding Box
        typedef struct {
            BoundingBox box;
            double yaw_rad;
        } OBB2;

        //Orientation Bounding Box
        typedef struct {
            double gc_x, gc_y, gc_z;
            double yaw_rad;
            double h, w, l;
        } GroundBox;

        typedef boost::geometry::model::polygon <
        boost::geometry::model::d2::point_xy<double>
        > Polygon;

        /**
         * @brief Object's 3D OBB to 2D ground box
         * @param object
         * @param gbox
         */
        static void toGroundBox(ObjectConstPtr object,
                                GroundBox* gbox)
        {
            gbox->gc_x = object->ground_center(0);
            gbox->gc_y = object->ground_center(1);
            gbox->gc_z = object->ground_center(2);
            gbox->yaw_rad = object->yaw_rad;
            gbox->h = object->height;
            gbox->w = object->width;
            gbox->l = object->length;
        }

        static void toGroundBox(const Eigen::Vector3f& center,
                                const Eigen::Vector3f& size,
                                const double& yaw,
                                GroundBox* gbox)
        {
            gbox->gc_x = center(0);
            gbox->gc_y = center(1);
            gbox->gc_z = center(2);
            gbox->yaw_rad = yaw;
            gbox->h = size(0);
            gbox->w = size(1);
            gbox->l = size(2);
        }

        /**
         * @brief Intersection-over-Union
         */
        static double bbIoU(const BoundingBox& box1, const BoundingBox& box2)
        {
            double box1_length = box1.x_max - box1.x_min;
            double box1_width = box1.y_max - box1.y_min;
            double area1 = box1_length * box1_width;

            double box2_length = box2.x_max - box2.x_min;
            double box2_width = box2.y_max - box2.y_min;
            double area2 = box2_length * box2_width;

            if (box1.x_min > box2.x_max) { return 0.0; }
            if (box1.y_min > box2.y_max) { return 0.0; }
            if (box1.x_max < box2.x_min) { return 0.0; }
            if (box1.y_max < box2.y_min) { return 0.0; }
            double inter_x = std::min(box1.x_max, box2.x_max) - std::max(box1.x_min, box2.x_min);
            double inter_y = std::min(box1.x_max, box2.x_max) - std::max(box1.x_min, box2.x_min);
            double intersection = inter_x * inter_y;
            return intersection / (area1 + area2 - intersection);
        }

        /*
         * @brief compute polygon of an oriented bounding box
         * @note Apollo's Object Coordinate
         *          |x
         *      C   |   D-----------
         *          |              |
         *  y---------------     length
         *          |              |
         *      B   |   A-----------
         */
        template<typename T>
        Polygon toPolygon(const T& g)
        {
            using namespace boost::numeric::ublas;
            using namespace boost::geometry;
            matrix<double> mref(2, 2);
            mref(0, 0) = cos(g.yaw_rad);
            mref(0, 1) = -sin(g.yaw_rad);
            mref(1, 0) = sin(g.yaw_rad);
            mref(1, 1) = cos(g.yaw_rad);

            matrix<double> corners(2, 4);
            //-------------(l/2,w/2)(l/2,-w/2)(-l/2,-w/2)(-l/2,w/2)
            double data[] = {g.l / 2, g.l / 2, -g.l / 2, -g.l / 2,
                             g.w / 2, -g.w / 2, -g.w / 2, g.w / 2};
            std::copy(data, data + 8, corners.data().begin());
            matrix<double> gc = prod(mref, corners);

            for (int i = 0; i < 4; ++i) {
                gc(0, i) += g.gc_x;
                gc(1, i) += g.gc_y;
            }

            double points[][2] = {{gc(0, 0), gc(1, 0)},
                                  {gc(0, 1), gc(1, 1)},
                                  {gc(0, 2), gc(1, 2)},
                                  {gc(0, 3), gc(1, 3)},
                                  {gc(0, 0), gc(1, 0)}};
            Polygon poly;
            append(poly, points);
            return poly;
        }

        /**
         * @brief Intersection-over-Union
         */
        static double groundBoxIoU(const GroundBox& box1, const GroundBox& box2)
        {
            using namespace boost::geometry;
            Polygon gp = toPolygon(box1);
            Polygon dp = toPolygon(box2);

            std::vector<Polygon> in, un;
            intersection(gp, dp, in);
            union_(gp, dp, un);

            double inter_area = in.empty() ? 0. : area(in.front());
            double union_area = area(un.front());

            double o = 0.;
            // union
            o = inter_area / union_area;
            // bbox_a
            //o = inter_area / area(dp);
            // bbox_b
            //o = inter_area / area(gp);
            return o;
        }

        static bool groundBoxInside(const GroundBox& box1, const GroundBox& box2)
        {
            using namespace boost::geometry;
            Polygon gp = toPolygon(box1);
            Polygon dp = toPolygon(box2);

            std::vector<Polygon> in;
            intersection(gp, dp, in);
            if (in.empty()) {
                return false;
            }
            else {
                double inter_area = area(in.front());
                double box1_area = area(gp);
                return abs(box1_area - inter_area) < EPSILON;
            }
        }

        /**
         * @brief check box1 is overlapping with box2
         *  true: box1 is inside box2 or box2 is inside box1
         *  true: IoU between box1 and box2 > threshold_IoU
         * @param box1
         * @param box2
         * @param threshold_IoU
         * @return
         */
        static bool groundBoxOverlap(const GroundBox& box1, const GroundBox& box2, double threshold_IoU)
        {
            if (groundBoxInside(box1, box2) || groundBoxInside(box2, box1)) {
                return true;
            }

            if (groundBoxIoU(box1, box2) > threshold_IoU) {
                return true;
            }

            return false;
        }

        /**
         * @brief predict object size and ground center based on object cloud and previous direction
         * @tparam PointT
         * @param cloud
         * @param direction
         * @param size
         * @param center
         */
        template<typename PointCloudPtrT>
        void computeBboxSizeCenter(PointCloudPtrT cloud, const Eigen::Vector3d& direction,
                                   Eigen::Vector3d *size, Eigen::Vector3d *center)
        {
            Eigen::Vector3d dir(direction[0], direction[1], 0);
            dir.normalize();
            Eigen::Vector3d ortho_dir(-dir[1], dir[0], 0.0);

            Eigen::Vector3d z_dir(dir.cross(ortho_dir));

            Eigen::Vector3d min_pt(DBL_MAX, DBL_MAX, DBL_MAX);
            Eigen::Vector3d max_pt(-DBL_MAX, -DBL_MAX, -DBL_MAX);
            Eigen::Vector3d loc_pt;
            for (size_t i = 0u; i < cloud->size(); ++i) {
                Eigen::Vector3d pt = Eigen::Vector3d(cloud->points[i].x,
                                                     cloud->points[i].y,
                                                     cloud->points[i].z);
                loc_pt[0] = pt.dot(dir);
                loc_pt[1] = pt.dot(ortho_dir);
                loc_pt[2] = pt.dot(z_dir);
                for (size_t j = 0u; j < 3; ++j) {
                    min_pt[j] = std::min(min_pt[j], loc_pt[j]);
                    max_pt[j] = std::max(max_pt[j], loc_pt[j]);
                }
            }

            *size = max_pt - min_pt;
            *center = dir * ((max_pt[0] + min_pt[0]) * 0.5) +
                      ortho_dir * ((max_pt[1] + min_pt[1]) * 0.5) + z_dir * min_pt[2];
        }
    }
}

#endif /* _BOUNDING_BOX_HPP_ */