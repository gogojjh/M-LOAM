#include "initial_extrinsics.h"
#include <cstdlib>
#include <ctime>

using namespace Eigen;

double EPSILON_R = 0.03;
double EPSILON_T = 0.1;

InitialExtrinsics::InitialExtrinsics() {}

void InitialExtrinsics::clearState()
{
    calib_ext_.clear();

    cov_rot_state_.clear();
    cov_pos_state_.clear();

    v_rot_cov_.clear();
    v_pos_cov_.clear();

    v_pose_.clear();
    indices_.clear();

    frame_cnt_ = 0;
    pose_cnt_ = 0;
}

void InitialExtrinsics::setParameter()
{
    for (size_t i = 0; i < NUM_OF_LASER; i++) calib_ext_.push_back(Pose(QBL[i], TBL[i], TDBL[i]));

    cov_rot_state_ = std::vector<bool>(NUM_OF_LASER, false);
    full_cov_rot_state_ = false;

    cov_pos_state_ = std::vector<bool>(NUM_OF_LASER, false);
    full_cov_pos_state_ = false;

    v_rot_cov_.resize(NUM_OF_LASER);
    v_pos_cov_.resize(NUM_OF_LASER);

    v_pose_.resize(NUM_OF_LASER);
    indices_.resize(NUM_OF_LASER);
}

bool InitialExtrinsics::setCovRotation(const size_t &idx)
{
    assert(idx < NUM_OF_LASER);
    cov_rot_state_[idx] = true;
    if (std::find(cov_rot_state_.begin(), cov_rot_state_.end(), false) == cov_rot_state_.end()) full_cov_rot_state_ = true;
}

bool InitialExtrinsics::setCovTranslation(const size_t &idx)
{
    assert(idx < NUM_OF_LASER);
    cov_pos_state_[idx] = true;
    if (std::find(cov_pos_state_.begin(), cov_pos_state_.end(), false) == cov_pos_state_.end()) full_cov_pos_state_ = true;
}

void InitialExtrinsics::addPose(const Pose &pose, const size_t &idx)
{
    assert(idx < NUM_OF_LASER);
    if (idx == 0) frame_cnt_++;
    v_pose_[idx].push_back(pose);
}

bool InitialExtrinsics::checkScrewMotion(const Pose &pose_ref, const Pose &pose_data)
{
    AngleAxisd ang_axis_ref(pose_ref.q_);
    AngleAxisd ang_axis_data(pose_data.q_);
    double r_dis = abs(ang_axis_ref.angle() - ang_axis_data.angle());
    double t_dis = abs(pose_ref.t_.dot(ang_axis_ref.axis()) - pose_data.t_.dot(ang_axis_data.axis()));
    // std::cout << "ref pose : " << pose_ref << std::endl;
    // std::cout << "data pose : " << pose_data << std::endl;
//    printf("r_dis: %f, t_dis: %f \n", r_dis, t_dis);
    v_rd_.push_back(r_dis);
    v_td_.push_back(t_dis);
    return (r_dis < EPSILON_R) && (t_dis < EPSILON_T);
}

bool InitialExtrinsics::calibExRotation(const size_t &idx_ref, const size_t &idx_data, Pose &calib_result)
{
    // -------------------------------
    // screw motion filter
    assert(idx_ref < NUM_OF_LASER);
    assert(idx_data < NUM_OF_LASER);
    assert(frame_cnt_ == v_pose_[idx_ref].size());
    if (!checkScrewMotion(*(v_pose_[idx_ref].end()-1), *(v_pose_[idx_data].end()-1)))
    {
        return false;
    } else
    {
        indices_[idx_data].push_back(frame_cnt_-1);
    }

    // -------------------------------
    // initial rotation
    TicToc t_calib_rot;
    pose_cnt_ = indices_[idx_data].size();
    // printf("********** pose_cnt %d\n", pose_cnt_);
    Eigen::MatrixXd A(pose_cnt_ * 4, 4); // a cumulative Q matrix
    A.setZero();
    for (auto i = 0; i < pose_cnt_; i++)
    {
        Pose &pose_ref = v_pose_[idx_ref][indices_[idx_data][i]];
        Pose &pose_data = v_pose_[idx_data][indices_[idx_data][i]];

        Eigen::Quaterniond r1 = pose_ref.q_;
        Eigen::Quaterniond r2 = calib_ext_[idx_data].q_ * pose_data.q_ * calib_ext_[idx_data].inverse().q_;
        double angular_distance = 180 / M_PI * r1.angularDistance(r2); // calculate delta_theta=|theta_1-theta_2|
        ROS_DEBUG("%d %f", i, angular_distance); 
        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;

        Eigen::Matrix4d L, R; // Q1 and Q2 to represent the quaternion representation
        double w = pose_ref.q_.w();
        Eigen::Vector3d q = pose_ref.q_.vec();
        L.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        w = pose_data.q_.w();
        q = pose_data.q_.vec();
        R.block<3, 3>(0, 0) = w * Eigen::Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        A.block<4, 4>(i * 4, 0) = huber * (L - R);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //cout << svd.singularValues().transpose() << endl;

    if (PLANAR_MOVEMENT)
    {
        // estimateRyx
        Eigen::Vector4d t1 = svd.matrixV().block<4,1>(0,2);
        Eigen::Vector4d t2 = svd.matrixV().block<4,1>(0,3);
        // solve constraint for q_yz: xy = -zw
        double s[2];
        if (!common::solveQuadraticEquation(t1(0) * t1(1) + t1(2) * t1(3),
                                        t1(0) * t2(1) + t1(1) * t2(0) + t1(2) * t2(3) + t1(3) * t2(2),
                                        t2(0) * t2(1) + t2(2) * t2(3),
                                        s[0], s[1]))
        {
            std::cout << "# ERROR: Quadratic equation cannot be solved due to negative determinant." << std::endl;
            return false;
        }
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > q_yx;
        q_yx.resize(2);
        double yaw[2];
        for (auto i = 0; i < 2; ++i)
        {
            double t = s[i] * s[i] * t1.dot(t1) + 2 * s[i] * t1.dot(t2) + t2.dot(t2);
            // solve constraint ||q_yx|| = 1
            double b = sqrt(1.0 / t);
            double a = s[i] * b;
            q_yx[i].coeffs() = a * t1 + b * t2; // [w x y z]
            Eigen::Vector3d euler_angles = q_yx[i].toRotationMatrix().eulerAngles(2, 1, 0);
            yaw[0] = euler_angles(0);
        }
        if (fabs(yaw[0]) < fabs(yaw[1]))
            calib_ext_[idx_data].q_ = q_yx[0];
        else
            calib_ext_[idx_data].q_ = q_yx[1];
    } else
    {
        Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
        calib_ext_[idx_data].q_ = Eigen::Quaterniond(x);
    }

    Eigen::Vector3d rot_cov = svd.singularValues().tail<3>(); // singular value
    v_rot_cov_[idx_data].push_back(rot_cov(1));
    printf("------------------- pose_cnt:%d, rot_cov:%f\n", pose_cnt_, rot_cov(1));
    if (pose_cnt_ >= WINDOW_SIZE && rot_cov(1) > 0.25) // converage, the second smallest sigular value
    {
        calib_result = calib_ext_[idx_data];
        // printf("calib ext rot: %fms\n", t_calib_rot.toc());
        return true;
    }
    else
    {
        return false;
    }
}

bool InitialExtrinsics::calibExTranslation(const size_t &idx_ref, const size_t &idx_data, Pose &calib_result)
{
    assert(idx_ref < NUM_OF_LASER);
    assert(idx_data < NUM_OF_LASER);
    assert(frame_cnt_ == v_pose_[idx_ref].size());
    if (!checkScrewMotion(*(v_pose_[idx_ref].end()-1), *(v_pose_[idx_data].end()-1)))
    {
        return false;
    } else
    {
        indices_[idx_data].push_back(frame_cnt_-1);
    }
    pose_cnt_ = indices_[idx_data].size();

    if (((PLANAR_MOVEMENT) && (calibExTranslationPlanar(idx_ref, idx_data))) ||
       ((!PLANAR_MOVEMENT) && (calibExTranslationNonPlanar(idx_ref, idx_data))))
    {
        calib_result = calib_ext_[idx_data];
        return true;
    } else
    {
        return false;
    }
}

bool InitialExtrinsics::calibExTranslationNonPlanar(const size_t &idx_ref, const size_t &idx_data)
{
    const Eigen::Quaterniond &q_zyx = calib_ext_[idx_data].q_;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(pose_cnt_ * 3, 3);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(pose_cnt_ * 3, 1);
    for (size_t i = 0; i < pose_cnt_; i++)
    {
        Pose &pose_ref = v_pose_[idx_ref][indices_[idx_data][i]];
        Pose &pose_data = v_pose_[idx_data][indices_[idx_data][i]];
        AngleAxisd ang_axis_ref(pose_ref.q_);
        AngleAxisd ang_axis_data(pose_data.q_);
        double t_dis = abs(pose_ref.t_.dot(ang_axis_ref.axis()) - pose_data.t_.dot(ang_axis_data.axis()));
        double huber = t_dis > 0.04 ? 0.04 / t_dis : 1.0;
        A.block<3, 3>(i * 3, 0) = huber * (pose_ref.q_.toRotationMatrix() - Eigen::Matrix3d::Identity());
        b.block<3, 1>(i * 3, 0) = q_zyx * pose_data.t_ - pose_ref.t_;
    }
    Eigen::Vector3d x;
    x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    calib_ext_[idx_data] = Pose(q_zyx, x);
    return true;
}

bool InitialExtrinsics::calibExTranslationPlanar(const size_t &idx_ref, const size_t &idx_data)
{
    const Eigen::Quaterniond &q_yx = calib_ext_[idx_data].q_;
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(pose_cnt_ * 2, 4);
    Eigen::MatrixXd w = Eigen::MatrixXd::Zero(pose_cnt_ * 2, 1);
    for (auto i = 0; i < pose_cnt_; ++i)
    {
        Pose &pose_ref = v_pose_[idx_ref][indices_[idx_data][i]];
        Pose &pose_data = v_pose_[idx_data][indices_[idx_data][i]];
        AngleAxisd ang_axis_ref(pose_ref.q_);
        AngleAxisd ang_axis_data(pose_data.q_);
        double t_dis = abs(pose_ref.t_.dot(ang_axis_ref.axis()) - pose_data.t_.dot(ang_axis_data.axis()));
        double huber = t_dis > 0.04 ? 0.04 / t_dis : 1.0;
        Eigen::Matrix2d J = pose_ref.q_.toRotationMatrix().block<2,2>(0, 0) - Eigen::Matrix2d::Identity();
        Eigen::Vector3d n = q_yx.toRotationMatrix().row(2);
        Eigen::Vector3d p = q_yx * (pose_data.t_ - pose_data.t_.dot(n) * n);
        Eigen::Matrix2d K;
        K << p(0), -p(1), p(1), p(0);
        G.block<2,2>(i * 2, 0) = huber * J;
        G.block<2,2>(i * 2, 2) = huber * K;
        w.block<2,1>(i * 2, 0) = pose_ref.t_.block<2,1>(0,0);
    }
    Eigen::Vector4d m = G.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(w);
    Eigen::Vector3d x(-m(0), -m(1), 0);
    double yaw = atan2(m(3), m(2));
    Eigen::Quaterniond q_z(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    calib_ext_[idx_data] = Pose(q_z*q_yx, x);
    return true;
}

void InitialExtrinsics::calibTimeDelay(const size_t &idx_ref, const size_t &idx_data)
{

}

void InitialExtrinsics::decomposeE(cv::Mat E,
                                 cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                 cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}

// save all poses of all lasers in initialization
void InitialExtrinsics::saveStatistics()
{
    try
    {
        ofstream fout(std::string(OUTPUT_FOLDER + "initialization.csv").c_str(), ios::out);
        fout.setf(ios::fixed, ios::floatfield);
        fout.precision(5);
        // orientation
        for (size_t i = 0; i < NUM_OF_LASER; i ++)
        {
            fout << 0 << ",";
            for (auto iter = v_pose_[i].begin(); iter != v_pose_[i].end(); iter++)
            {
                fout << Eigen::AngleAxisd(iter->q_).angle() << ",";
                if (iter == v_pose_[i].end() - 1) fout << std::endl;
            }
        }
        // rot_cov
        for (size_t i = 0; i < NUM_OF_LASER; i ++)
        {
            fout << 0 << ",";
            for (auto iter = v_rot_cov_[i].begin(); iter != v_rot_cov_[i].end(); iter++)
            {
                fout << *iter << ",";
                if (iter == v_rot_cov_[i].end() - 1) fout << std::endl;
            }
        }
        // pos_cov
        for (size_t i = 0; i < NUM_OF_LASER; i ++)
        {
            fout << 0 << ",";
            for (auto iter = v_pos_cov_[i].begin(); iter != v_pos_cov_[i].end(); iter++)
            {
                fout << *iter << ",";
                if (iter == v_pos_cov_[i].end() - 1) fout << std::endl;
            }
        }
        fout.close();
    }
    catch (const std::exception &ex)
    {
        std::cout << "Exception was thrown when save initial extrinsics: " << ex.what() << std::endl;
    }
    return;
}
