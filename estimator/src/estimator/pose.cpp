/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose.h"

Pose Pose::poseTransform(const Pose &pose1, const Pose &pose2)
{
    // t12 = t1 + q1 * t2;
    // q12 = q1 * q2;
    return Pose(pose1.q_*pose2.q_, pose1.q_*pose2.t_+pose1.t_);
}

Pose Pose::inverse()
{
    return Pose(q_.inverse(), -(q_.inverse()*t_));
}

Pose Pose::operator * (const Pose &pose)
{
    return Pose(q_*pose.q_, q_*pose.t_+t_);
}

ostream & operator << (ostream &out, const Pose &pose)
{
    out << "t: [" << pose.t_.transpose() << "], q: ["
        << pose.q_.x() << " " << pose.q_.y() << " " << pose.q_.z() << " " << pose.q_.w() << "], td: "
        << pose.td_;
    return out;
}
