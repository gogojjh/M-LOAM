#ifndef _COMMON_HPP_
#define _COMMON_HPP_

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>              /* pcl::io::savePCDFileASCII */
#include <pcl/common/centroid.h>        /* pcl::compute3DCentroid */
#include <pcl/common/transforms.h>      /* pcl::transformPointCloud */

#include <string>
#include <math.h>                       /* sqrt, pow */
#include <Eigen/Core>

#include "./types/type.h"
#include "./algos/math.hpp"

namespace common
{
    //float precision
    const float EPSILON = 1e-9;

    //----------------------------------- sort compare function
    template <typename PointType>
    bool sortByAxisXAsc(PointType p1, PointType p2)
    {
        return p1.x < p2.x;
    }

    template <typename PointType>
    bool sortByAxisZAsc(PointType p1, PointType p2)
    {
        return p1.z < p2.z;
    }

    template <typename ObjType>
    bool sortByObjSizeDesc(ObjType obj1, ObjType obj2)
    {
        return obj1->cloud->size() > obj2->cloud->size();
    }

    /// \brief Utility function for swapping two values.
    template<typename T>
    bool swap_if_gt(T& a, T& b)
    {
        if (a > b) {
            std::swap(a, b);
            return true;
        }
        return false;
    }

    //----------------------------------- *.pcd
    static void savePCDModel(PointICloudConstPtr pc, const std::string& model_name)
    {
        //std::string pcd_model_file = "model.pcd";
        pcl::io::savePCDFileASCII(model_name, *pc);
        ROS_INFO_STREAM("PCD Model " << model_name << " saved.");
    }

    static void loadPCDModel(PointICloudPtr pc, const std::string& model_name)
    {
        pcl::io::loadPCDFile<PointI>(model_name, *pc);
        ROS_INFO_STREAM("PCD Model " << model_name << " loaded.");
    }

    static bool loadPCDModel(PointCloudPtr pc, Eigen::Affine3f& model2world)
    {
        std::string pcd_model_file = "/home/gary/Workspace/intern_ws/pcl_learning/model.pcd";
        if (pcl::io::loadPCDFile<Point>(pcd_model_file, *pc) == -1) {
            return false;
        } else {
            ROS_INFO_STREAM("PCD Model " << pcd_model_file << " loaded");

            model2world = Eigen::Affine3f::Identity();
            Eigen::Vector4f model_centroid;
            pcl::compute3DCentroid<Point>(*pc, model_centroid);
            model2world.translation().matrix() = Eigen::Vector3f(model_centroid[0], model_centroid[1], model_centroid[2]);
            pcl::transformPointCloud(*pc, *pc, model2world.inverse());

            return true;
        }
    }

    /**
     * @brief convert PointI cloud in indices to PointD cloud
     * @param cloud
     * @param indices
     * @param trans_cloud
     */
    static void convertPointCloud(PointICloudPtr icloud,
                                  const std::vector<int>& indices,
                                  PointDCloud* dcloud)
    {
        if (dcloud->size() != indices.size()) {
            dcloud->resize(indices.size());
        }
        for (size_t i = 0u; i < indices.size(); ++i) {
            const PointI& p = icloud->at(indices[i]);
            Eigen::Vector3d v(p.x, p.y, p.z);
            PointD& tp = dcloud->at(i);
            tp.x = v.x();
            tp.y = v.y();
            tp.z = v.z();
            tp.intensity = p.intensity;
        }
    }

    //----------------------------------- print information
    static void displayPerformances(unsigned int tp, unsigned int tn,
                                    unsigned int fp, unsigned int fn)
    {
        ROS_INFO_STREAM("TP: " << tp << ", TN: " << tn <<
                        ", FP: " << fp << ", FN: " << fn << ".");

        const double true_positive_rate = double(tp) / double(tp + fn);
        const double true_negative_rate = double(tn) / double(fp + tn);
        const double false_positive_rate = 1.0 - true_negative_rate;

        ROS_INFO_STREAM("Accuracy (ACC): " << double(tp + tn) /
                                           double(tp + fp + tn + fn));
        ROS_INFO_STREAM("Sensitivity (TPR): " << true_positive_rate);
        ROS_INFO_STREAM("Specificity (TNR): " << true_negative_rate);
        ROS_INFO_STREAM("Precision: " << double(tp) / double(tp + fp));
        ROS_INFO_STREAM("Positive likelyhood ratio: " << true_positive_rate / false_positive_rate);
    }

    static void displayGroundTruth(const Eigen::Vector3f& center,
                                   const Eigen::Vector3f& size,
                                   double yaw_rad)
    {
        ROS_INFO("-------- Bounding Box --------");
        ROS_INFO_STREAM("\tSize: " << size(0) << " x " << size(1) << " x " << size(2));
        ROS_INFO_STREAM("\tCenter: (" << center(0) << ", " << center(1) << ", " << center(2) << ")");
        ROS_INFO_STREAM("\tYaw(rad): " << yaw_rad);
        ROS_INFO("-------- Bounding Box --------");
    }

    template <typename VolumetricModelType>
    static void displayModelInfo(const VolumetricModelType& model)
    {
        ROS_INFO("Model:\t length[%.2f..%.2f]/width[%.2f..%.2f]/height[%.2f..%.2f]\n", model.l_min, model.l_max,
                                                                                       model.w_min, model.w_max,
                                                                                       model.h_min, model.h_max);
    }

    //----------------------------------- math utils
    template <typename PointType>
    void removeROIPointCloud(const pcl::PointCloud<PointType> &cloud_in,
                             pcl::PointCloud<PointType> &cloud_out,
                             float thres,
                             std::string mode)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;
        if (!mode.compare("inside"))
        {
            for (size_t i = 0; i < cloud_in.points.size(); ++i)
            {
                if (sqrSum(cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z) < thres * thres)
                    continue;
                cloud_out.points[j] = cloud_in.points[i];
                j++;
            }
        }
        else if (!mode.compare("outside"))
        {
            for (size_t i = 0; i < cloud_in.points.size(); ++i)
            {
                if (sqrSum(cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z) > thres * thres)
                    continue;
                cloud_out.points[j] = cloud_in.points[i];
                j++;
            }
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }
        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }
}

#endif /* _COMMON_HPP_ */
