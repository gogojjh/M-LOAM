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

#define PCL_NO_PRECOMPILE

#ifndef POINTWITHCOV_H
#define POINTWITHCOV_H

#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <pcl/impl/instantiate.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/filters/impl/voxel_grid_covariance.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <bitset>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/fold.hpp>
#include <boost/mpl/vector.hpp>

// pcl point type: https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/impl/point_types.hpp
namespace pcl
{
    struct EIGEN_ALIGN16 _PointXYZIWithCov
    {
        PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
        float intensity;
        float cov_vec[6]; // cxx, cxy, cxz, cyy, cyz, czz
        float cov_trace;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    struct PointXYZIWithCov: public _PointXYZIWithCov
    {
        inline PointXYZIWithCov()
        {
            x = y = z = 0.0f; intensity = 0;
            cov_vec[0] = cov_vec[1] = cov_vec[2] = cov_vec[3] = cov_vec[4] = cov_vec[5] = 0.0f;
            cov_trace = 0.0f;
        }

        inline PointXYZIWithCov(float _x, float _y, float _z, float _intensity,
                float _cov_xx, float _cov_xy, float _cov_xz,
                float _cov_yy, float _cov_yz, float _cov_zz)
        {
            x = _x; y = _y; z = _z; intensity = _intensity;
            cov_vec[0] = _cov_xx;
            cov_vec[1] = _cov_xy;
            cov_vec[2] = _cov_xz;
            cov_vec[3] = _cov_yy;
            cov_vec[4] = _cov_yz;
            cov_vec[5] = _cov_zz;
            cov_trace = _cov_xx + _cov_yy + _cov_zz;
        }

        inline PointXYZIWithCov(const _PointXYZIWithCov &p)
        {
            x = p.x; y = p.y; z = p.z; intensity = p.intensity;
            cov_vec[0] = p.cov_vec[0];
            cov_vec[1] = p.cov_vec[1];
            cov_vec[2] = p.cov_vec[2];
            cov_vec[3] = p.cov_vec[3];
            cov_vec[4] = p.cov_vec[4];
            cov_vec[5] = p.cov_vec[5];
            cov_trace = p.cov_trace;
        }

        inline PointXYZIWithCov(const PointXYZI &p, const Eigen::Matrix3f &cov_matrix)
        {
            x = p.x; y = p.y; z = p.z; intensity = p.intensity;
            cov_vec[0] = cov_matrix(0, 0);
            cov_vec[1] = cov_matrix(0, 1);
            cov_vec[2] = cov_matrix(0, 2);
            cov_vec[3] = cov_matrix(1, 1);
            cov_vec[4] = cov_matrix(1, 2);
            cov_vec[5] = cov_matrix(2, 2);
            cov_trace = cov_matrix(0, 0) + cov_matrix(1, 1) + cov_matrix(2, 2);
        }
        friend std::ostream &operator << (std::ostream &out, const PointXYZIWithCov &p);
    };

    inline std::ostream &operator << (std::ostream &out, const PointXYZIWithCov &p)
    {
        out << "(" << p.x << ", " << p.y << ", " << p.z << ", " << p.intensity << ", "
            << p.cov_vec[0] << ", " << p.cov_vec[1] << ", " << p.cov_vec[2] << ", "
            << p.cov_vec[3] << ", " << p.cov_vec[4] << ", "<< p.cov_vec[5] << ", " << p.cov_trace << ")";
        return out;
    }
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_PointXYZIWithCov,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (float, cov_vec[0], cov_xx)
                                 (float, cov_vec[1], cov_xy)
                                 (float, cov_vec[2], cov_xz)
                                 (float, cov_vec[3], cov_yy)
                                 (float, cov_vec[4], cov_yz)
                                 (float, cov_vec[5], cov_zz)
                                 (float, cov_trace, cov_trace)
)

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZIWithCov, pcl::_PointXYZIWithCov)

namespace common
{
    typedef pcl::PointXYZIWithCov PointIWithCov;
    typedef pcl::PointCloud<PointIWithCov> PointICovCloud;
    typedef PointICovCloud::Ptr PointICovCloudPtr;
    typedef PointICovCloud::ConstPtr PointICovCloudConstPtr;

    void removeCov(const pcl::PointXYZIWithCov &pi, pcl::PointXYZI &po)
    {
        po.x = pi.x; po.y = pi.y; po.z = pi.z;
        po.intensity = pi.intensity;
    }

    pcl::PointXYZI removeCov(const pcl::PointXYZIWithCov &pi)
    {
        pcl::PointXYZI po;
        po.x = pi.x; po.y = pi.y; po.z = pi.z;
        po.intensity = pi.intensity;
        return po;
    }

    void appendCov(const pcl::PointXYZI &pi, pcl::PointXYZIWithCov &po, const Eigen::Matrix3d &cov_matrix)
    {
        po.x = pi.x; po.y = pi.y; po.z = pi.z; po.intensity = pi.intensity;
        po.cov_vec[0] = cov_matrix(0, 0);
        po.cov_vec[1] = cov_matrix(0, 1);
        po.cov_vec[2] = cov_matrix(0, 2);
        po.cov_vec[3] = cov_matrix(1, 1);
        po.cov_vec[4] = cov_matrix(1, 2);
        po.cov_vec[5] = cov_matrix(2, 2);
        po.cov_trace = cov_matrix(0, 0) + cov_matrix(1, 1) + cov_matrix(2, 2);
    }

    pcl::PointXYZIWithCov appendCov(const pcl::PointXYZI &pi, const Eigen::Matrix3d &cov_matrix)
    {
        pcl::PointXYZIWithCov po;
        po.x = pi.x; po.y = pi.y; po.z = pi.z; po.intensity = pi.intensity;
        po.cov_vec[0] = cov_matrix(0, 0);
        po.cov_vec[1] = cov_matrix(0, 1);
        po.cov_vec[2] = cov_matrix(0, 2);
        po.cov_vec[3] = cov_matrix(1, 1);
        po.cov_vec[4] = cov_matrix(1, 2);
        po.cov_vec[5] = cov_matrix(2, 2);
        po.cov_trace = cov_matrix(0, 0) + cov_matrix(1, 1) + cov_matrix(2, 2);
        return po;
    }

    void updateCov(const pcl::PointXYZIWithCov &pi, pcl::PointXYZIWithCov &po, const Eigen::Matrix3d &cov_matrix)
    {
        po.x = pi.x;
        po.y = pi.y;
        po.z = pi.z;
        po.intensity = pi.intensity;
        po.cov_vec[0] = cov_matrix(0, 0);
        po.cov_vec[1] = cov_matrix(0, 1);
        po.cov_vec[2] = cov_matrix(0, 2);
        po.cov_vec[3] = cov_matrix(1, 1);
        po.cov_vec[4] = cov_matrix(1, 2);
        po.cov_vec[5] = cov_matrix(2, 2);
        po.cov_trace = cov_matrix(0, 0) + cov_matrix(1, 1) + cov_matrix(2, 2);
    }

    void updateCov(pcl::PointXYZIWithCov &po, const Eigen::Matrix3d &cov_matrix)
    {
        po.cov_vec[0] = cov_matrix(0, 0);
        po.cov_vec[1] = cov_matrix(0, 1);
        po.cov_vec[2] = cov_matrix(0, 2);
        po.cov_vec[3] = cov_matrix(1, 1);
        po.cov_vec[4] = cov_matrix(1, 2);
        po.cov_vec[5] = cov_matrix(2, 2);
        po.cov_trace = cov_matrix(0, 0) + cov_matrix(1, 1) + cov_matrix(2, 2);
    }

    void extractCov(const pcl::PointXYZIWithCov &p, Eigen::Matrix3d &cov_matrix)
    {
        cov_matrix.setZero();
        cov_matrix(0, 0) = p.cov_vec[0],
        cov_matrix(0, 1) = p.cov_vec[1],
        cov_matrix(0, 2) = p.cov_vec[2],
        cov_matrix(1, 0) = p.cov_vec[1],
        cov_matrix(1, 1) = p.cov_vec[3],
        cov_matrix(1, 2) = p.cov_vec[4],
        cov_matrix(2, 0) = p.cov_vec[2],
        cov_matrix(2, 1) = p.cov_vec[4],
        cov_matrix(2, 2) = p.cov_vec[5];
    }
}

#endif


//
