#define PCL_NO_PRECOMPILE

#ifndef POINTWITHCOV_H
#define POINTWITHCOV_H

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <bitset>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/fold.hpp>
#include <boost/mpl/vector.hpp>

namespace pcl
{
    // standard PCL style: http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
    // http://wiki.ros.org/pcl_ros/cturtle
    // new point type with covariance
    struct EIGEN_ALIGN16 _PointIWithCov
    {
        PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
        float intensity;
        float cov_vec[6]; // cxx, cxy, cxz, cyy, cyz, czz
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    } EIGEN_ALIGN16;

    struct PointIWithCov: public _PointIWithCov
    {
        inline PointIWithCov(const _PointIWithCov &p)
        {
            x = p.x; y = p.y; z = p.z; intensity = p.intensity;
            cov_vec[0] = p.cov_vec[0];
            cov_vec[1] = p.cov_vec[1];
            cov_vec[2] = p.cov_vec[2];
            cov_vec[3] = p.cov_vec[3];
            cov_vec[4] = p.cov_vec[4];
            cov_vec[5] = p.cov_vec[5];
        }

        inline PointIWithCov()
        {
            x = y = z = 0.0f;
            intensity = 0.0f;
            cov_vec[0] = 0;
            cov_vec[1] = 0;
            cov_vec[2] = 0;
            cov_vec[3] = 0;
            cov_vec[4] = 0;
            cov_vec[5] = 0;
        }
        inline PointIWithCov(const PointXYZI &p, const Eigen::Matrix3f &cov_matrix)
        {
            x = p.x; y = p.y; z = p.z; intensity = p.intensity;
            cov_vec[0] = cov_matrix(0, 0);
            cov_vec[1] = cov_matrix(0, 1);
            cov_vec[2] = cov_matrix(0, 2);
            cov_vec[3] = cov_matrix(1, 1);
            cov_vec[4] = cov_matrix(1, 2);
            cov_vec[5] = cov_matrix(2, 2);
        }
        friend std::ostream &operator << (std::ostream &out, const PointIWithCov &p);
    };

    std::ostream &operator << (std::ostream &out, const PointIWithCov &p)
    {
        out << "(" << p.x << ", " << p.y << ", " << p.z << ", " << p.intensity << ", "
            << p.cov_vec[0] << ", " << p.cov_vec[1] << ", " << p.cov_vec[2] << ", "
            << p.cov_vec[3] << ", " << p.cov_vec[4] << ", "<< p.cov_vec[5] << ")" << std::endl;
        return out;
    }

    inline PointXYZI removeCov(const PointIWithCov &pi)
    {
        PointXYZI po;
        po.x = pi.x; po.y = pi.y; po.z = pi.z;
        po.intensity = pi.intensity;
    }

    inline PointIWithCov appendCov(const PointXYZI &pi, const Eigen::Matrix3f &cov_matrix)
    {
        PointIWithCov po;
        po.x = pi.x; po.y = pi.y; po.z = pi.z; po.intensity = pi.intensity;
        po.cov_vec[0] = cov_matrix(0, 0);
        po.cov_vec[1] = cov_matrix(0, 1);
        po.cov_vec[2] = cov_matrix(0, 2);
        po.cov_vec[3] = cov_matrix(1, 1);
        po.cov_vec[4] = cov_matrix(1, 2);
        po.cov_vec[5] = cov_matrix(2, 2);
    }
}

// here we assume a xyz + "covariance" (as fields)
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointIWithCov,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float[6], cov_vec, cov)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointIWithCov, pcl::_PointIWithCov)

namespace common
{
    typedef pcl::PointCloud<pcl::PointIWithCov> PointICovCloud;
    typedef PointICovCloud::Ptr PointICovCloudPtr;
    typedef PointICovCloud::ConstPtr PointICovCloudConstPtr;
}

#endif





//
