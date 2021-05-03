#pragma once

#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/pose.h"

// ****************** Barfoot's method on associating uncertainty on SE3
inline Eigen::Matrix<double, 6, 6> adjointMatrix(const Eigen::Matrix4d &T)
{
    Eigen::Matrix<double, 6, 6> AdT = Eigen::Matrix<double, 6, 6>::Zero();
    AdT.topLeftCorner<3, 3>() = T.topLeftCorner<3, 3>();
    AdT.topRightCorner<3, 3>() = Utility::skewSymmetric(T.topRightCorner<3, 1>()) * T.topLeftCorner<3, 3>();
    AdT.bottomRightCorner<3, 3>() = T.topLeftCorner<3, 3>();
    return AdT;
}

inline Eigen::Matrix3d covop1(const Eigen::Matrix3d &B)
{
    Eigen::Matrix3d A = -B.trace() * Eigen::Matrix3d::Identity() + B;
    return A;
}

inline Eigen::Matrix3d covop2(const Eigen::Matrix3d &B, const Eigen::Matrix3d &C)
{
    Eigen::Matrix3d A = covop1(B) * covop1(C) + covop1(C * B);
    return A;
}

// fixed: topLeftCorner<3, 3>()
// dynamic: topLeftCorner(3, 3)
inline void compoundPoseWithCov(const Pose &pose_1, const Eigen::Matrix<double, 6, 6> &cov_1,
                                const Pose &pose_2, const Eigen::Matrix<double, 6, 6> &cov_2,
                                Pose &pose_cp, Eigen::Matrix<double, 6, 6> &cov_cp, 
                                const int &method = 2)
{
    pose_cp.q_ = pose_1.q_ * pose_2.q_;
    pose_cp.t_ = pose_1.q_ * pose_2.t_ + pose_1.t_;
    Eigen::Matrix<double, 6, 6> AdT1 = adjointMatrix(pose_1.T_); // the adjoint matrix of T1
    Eigen::Matrix<double, 6, 6> cov_2_prime = AdT1 * cov_2 * AdT1.transpose();
    if (method == 1)
    {
        cov_cp = cov_1 + cov_2_prime;
    }
    else if (method == 2)
    {
        Eigen::Matrix3d cov_1_rr = cov_1.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_1_rp = cov_1.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_1_pp = cov_1.bottomRightCorner<3, 3>();

        Eigen::Matrix3d cov_2_rr = cov_2_prime.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_2_rp = cov_2_prime.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_2_pp = cov_2_prime.bottomRightCorner<3, 3>();

        Eigen::Matrix<double, 6, 6> A1 = Eigen::Matrix<double, 6, 6>::Zero();
        A1.topLeftCorner<3, 3>() = covop1(cov_1_pp);
        A1.topRightCorner<3, 3>() = covop1(cov_1_rp + cov_1_rp.transpose());
        A1.bottomRightCorner<3, 3>() = covop1(cov_1_pp);

        Eigen::Matrix<double, 6, 6> A2 = Eigen::Matrix<double, 6, 6>::Zero();
        A2.topLeftCorner<3, 3>() = covop1(cov_2_pp);
        A2.topRightCorner<3, 3>() = covop1(cov_2_rp + cov_2_rp.transpose());
        A2.bottomRightCorner<3, 3>() = covop1(cov_2_pp);

        Eigen::Matrix3d Brr = covop2(cov_1_pp, cov_2_rr) + covop2(cov_1_rp.transpose(), cov_2_rp) +
                              covop2(cov_1_rp, cov_2_rp.transpose()) + covop2(cov_1_rr, cov_2_pp);
        Eigen::Matrix3d Brp = covop2(cov_1_pp, cov_2_rp.transpose()) + covop2(cov_1_rp.transpose(), cov_2_pp);
        Eigen::Matrix3d Bpp = covop2(cov_1_pp, cov_2_pp);
        Eigen::Matrix<double, 6, 6> B = Eigen::Matrix<double, 6, 6>::Zero();
        B.topLeftCorner<3, 3>() = Brr;
        B.topRightCorner<3, 3>() = Brp;
        B.bottomLeftCorner<3, 3>() = Brp.transpose();
        B.bottomRightCorner<3, 3>() = Bpp;

        cov_cp = cov_1 + cov_2_prime + (A1 * cov_2_prime + cov_2_prime * A1.transpose() + A2 * cov_1 + cov_1 * A2.transpose()) / 12 + B / 4;
    }
    else
    {
        printf("[compoundPoseWithCov] No %dth method !\n", method);
        cov_cp.setZero();
    }
    // std::cout << pose_1 << std::endl << cov_1 << std::endl;
    // std::cout << pose_2 << std::endl << cov_2 << std::endl;
    // std::cout << pose_cp << std::endl << cov_cp << std::endl;
    // exit(EXIT_FAILURE);
}

// fixed: topLeftCorner<3, 3>()
// dynamic: topLeftCorner(3, 3)
inline void compoundPoseWithCov(const Pose &pose_1,
                                const Pose &pose_2,
                                Pose &pose_cp,
                                const int &method = 2)
{
    const Eigen::Matrix<double, 6, 6> &cov_1 = pose_1.cov_;
    const Eigen::Matrix<double, 6, 6> &cov_2 = pose_2.cov_;
    pose_cp.q_ = pose_1.q_ * pose_2.q_;
    pose_cp.t_ = pose_1.q_ * pose_2.t_ + pose_1.t_;
    pose_cp.update();

    Eigen::Matrix<double, 6, 6> AdT1 = adjointMatrix(pose_1.T_); // the adjoint matrix of T1
    Eigen::Matrix<double, 6, 6> cov_2_prime = AdT1 * cov_2 * AdT1.transpose();
    if (method == 1)
    {
        pose_cp.cov_ = cov_1 + cov_2_prime;
    }
    else if (method == 2)
    {
        Eigen::Matrix3d cov_1_rr = cov_1.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_1_rp = cov_1.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_1_pp = cov_1.bottomRightCorner<3, 3>();

        Eigen::Matrix3d cov_2_rr = cov_2_prime.topLeftCorner<3, 3>();
        Eigen::Matrix3d cov_2_rp = cov_2_prime.topRightCorner<3, 3>();
        Eigen::Matrix3d cov_2_pp = cov_2_prime.bottomRightCorner<3, 3>();

        Eigen::Matrix<double, 6, 6> A1 = Eigen::Matrix<double, 6, 6>::Zero();
        A1.topLeftCorner<3, 3>() = covop1(cov_1_pp);
        A1.topRightCorner<3, 3>() = covop1(cov_1_rp + cov_1_rp.transpose());
        A1.bottomRightCorner<3, 3>() = covop1(cov_1_pp);

        Eigen::Matrix<double, 6, 6> A2 = Eigen::Matrix<double, 6, 6>::Zero();
        A2.topLeftCorner<3, 3>() = covop1(cov_2_pp);
        A2.topRightCorner<3, 3>() = covop1(cov_2_rp + cov_2_rp.transpose());
        A2.bottomRightCorner<3, 3>() = covop1(cov_2_pp);

        Eigen::Matrix3d Brr = covop2(cov_1_pp, cov_2_rr) + covop2(cov_1_rp.transpose(), cov_2_rp) + covop2(cov_1_rp, cov_2_rp.transpose()) + covop2(cov_1_rr, cov_2_pp);
        Eigen::Matrix3d Brp = covop2(cov_1_pp, cov_2_rp.transpose()) + covop2(cov_1_rp.transpose(), cov_2_pp);
        Eigen::Matrix3d Bpp = covop2(cov_1_pp, cov_2_pp);
        Eigen::Matrix<double, 6, 6> B = Eigen::Matrix<double, 6, 6>::Zero();
        B.topLeftCorner<3, 3>() = Brr;
        B.topRightCorner<3, 3>() = Brp;
        B.bottomLeftCorner<3, 3>() = Brp.transpose();
        B.bottomRightCorner<3, 3>() = Bpp;

        pose_cp.cov_ = cov_1 + cov_2_prime + (A1 * cov_2_prime + cov_2_prime * A1.transpose() + A2 * cov_1 + cov_1 * A2.transpose()) / 12 + B / 4;
    }
    else
    {
        printf("[compoundPoseWithCov] No %dth method !\n", method);
        pose_cp.cov_.setZero();
    }
    // std::cout << pose_1 << std::endl << pose_1.cov_ << std::endl;
    // std::cout << pose_2 << std::endl << pose_2.cov_ << std::endl;
    // std::cout << pose_cp << std::endl << pose_cp.cov_ << std::endl;
    // exit(EXIT_FAILURE);
}

// pointToFS turns a 4x1 homogeneous point into a special 4x6 matrix
inline Eigen::Matrix<double, 4, 6> pointToFS(const Eigen::Vector4d &point)
{
    Eigen::Matrix<double, 4, 6> G = Eigen::Matrix<double, 4, 6>::Zero();
    G.block<3, 3>(0, 0) = point(3) * Eigen::Matrix3d::Identity();
    G.block<3, 3>(0, 3) = -Utility::skewSymmetric(point.block<3, 1>(0, 0));
    return G;
}

/*
 * pi: the original point for evaluating uncertainty
 * cov_point: associated covariance of the point
 * pose: pose
 * cov_pose: associated covariance of the pose
 */
template <typename PointType>
inline void evalPointUncertainty(const PointType &pi,
                                 Eigen::Matrix3d &cov_point,
                                 const Pose &pose,
                                 const Eigen::Matrix<double, 6, 6> &cov_pose)
{
    // THETA: diag(P, Phi, Z) includes the translation, rotation, measurement uncertainty
    Eigen::Matrix<double, 9, 9> cov_input = Eigen::Matrix<double, 9, 9>::Zero();
    cov_input.topLeftCorner<6, 6>() = cov_pose;
    cov_input.bottomRightCorner<3, 3>() = COV_MEASUREMENT;

    Eigen::Vector4d point_curr(pi.x, pi.y, pi.z, 1);
    Eigen::Matrix4d T = pose.T_;
    // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 4, 3> D;
    D << 1, 0, 0,
         0, 1, 0,
         0, 0, 1,
         0, 0, 0;
    Eigen::Matrix<double, 4, 9> G = Eigen::Matrix<double, 4, 9>::Zero();
    G.block<4, 6>(0, 0) = pointToFS(T * point_curr);
    G.block<4, 3>(0, 6) = T * D;
    cov_point = Eigen::Matrix4d(G * cov_input * G.transpose()).topLeftCorner<3, 3>(); // 3x3
    // std::cout << cov_input << std::endl;
    // std::cout << G << std::endl;
    // std::cout << "evalUncertainty:" << std::endl
    //           << point_curr.transpose() << std::endl
    //           << cov_point << std::endl;
    // exit(EXIT_FAILURE);
}

template <typename PointType>
inline void evalPointUncertainty(const PointType &pi, Eigen::Matrix3d &cov_point, const Pose &pose)
{
    // THETA: diag(P, Phi, Z) includes the translation, rotation, measurement uncertainty
    Eigen::Matrix<double, 9, 9> cov_input = Eigen::Matrix<double, 9, 9>::Zero();
    cov_input.topLeftCorner<6, 6>() = pose.cov_;
    cov_input.bottomRightCorner<3, 3>() = COV_MEASUREMENT;

    Eigen::Vector4d point_curr(pi.x, pi.y, pi.z, 1);
    Eigen::Matrix4d T = pose.T_;
    // Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 4, 3> D;
    D << 1, 0, 0,
         0, 1, 0,
         0, 0, 1,
         0, 0, 0;
    Eigen::Matrix<double, 4, 9> G = Eigen::Matrix<double, 4, 9>::Zero();
    G.block<4, 6>(0, 0) = pointToFS(T * point_curr);
    G.block<4, 3>(0, 6) = T * D;
    cov_point = Eigen::Matrix4d(G * cov_input * G.transpose()).topLeftCorner<3, 3>(); // 3x3
}



