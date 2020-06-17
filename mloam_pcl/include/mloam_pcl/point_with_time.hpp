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

#ifndef POINTWITHTIME_HPP
#define POINTWITHTIME_HPP

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

#include <bitset>
#include <boost/mpl/contains.hpp>
#include <boost/mpl/fold.hpp>
#include <boost/mpl/vector.hpp>

// pcl point type: https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/impl/point_types.hpp
namespace pcl
{
    struct EIGEN_ALIGN16 _PointXYZIWithTime
    {
        PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
        float intensity;
        float timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    struct PointXYZIWithTime: public _PointXYZIWithTime
    {
        inline PointXYZIWithTime()
        {
            x = y = z = 0.0f; intensity = 0;
            timestamp = 0.0f;
        }

        inline PointXYZIWithTime(float _x, float _y, float _z, 
            float _intensity, float _timestamp)
        {
            x = _x; y = _y; z = _z; 
            intensity = _intensity;
            timestamp = _timestamp;
        }

        inline PointXYZIWithTime(const _PointXYZIWithTime &p)
        {
            x = p.x; y = p.y; z = p.z; 
            intensity = p.intensity;
            timestamp = p.timestamp;
        }

        friend std::ostream &operator << (std::ostream &out, const PointXYZIWithTime &p);
    };

    inline std::ostream &operator << (std::ostream &out, const PointXYZIWithTime &p)
    {
        out.precision(5);
        out << "(" << p.x << ", " << p.y << ", " << p.z << ", " 
            << p.intensity << ", " << p.timestamp << ")";
        return out;
    }
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_PointXYZIWithTime,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (float, timestamp, timestamp)
)

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZIWithTime, pcl::_PointXYZIWithTime)

namespace common
{
    typedef pcl::PointXYZIWithTime PointIWithTime;
    typedef pcl::PointCloud<PointIWithTime> PointITimeCloud;
    typedef PointITimeCloud::Ptr PointITimeCloudPtr;
    typedef PointITimeCloud::ConstPtr PointITimeCloudConstPtr;
}

#endif


//
