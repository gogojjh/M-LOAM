#include <Eigen/Dense>
#include <iostream>

static const Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
        q(2), typename Derived::Scalar(0), -q(0),
        -q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
{
    //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
    //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
    //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
    //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
    return q;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuatMatrix(const Eigen::QuaternionBase<Derived> &p)
{
    Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
    Eigen::Matrix<typename Derived::Scalar, 3, 1> vp = p.vec();
    typename Derived::Scalar p4 = p.w();
    m.block(0, 0, 3, 3) << p4 * I3x3 - SkewSymmetric(vp);
    m.block(3, 0, 1, 3) << -vp.transpose();
    m.block(0, 3, 3, 1) << vp;
    m(3, 3) = p4;
    return m;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(const Eigen::QuaternionBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
    Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
    typename Derived::Scalar q4 = q.w();
    m.block(0, 0, 3, 3) << q4 * I3x3 - SkewSymmetric(vq);
    m.block(3, 0, 1, 3) << -vq.transpose();
    m.block(0, 3, 3, 1) << vq;
    m(3, 3) = q4;
    return m;
}

int main(int argc, char **argv)
{
    Eigen::Quaterniond q1(0.96593, 0, 0, 0.25882);
    {
        Eigen::Quaterniond q2(0.95155, 0.038135, 0.18931, 0.2393);
        Eigen::Quaterniond q = q1 * q2;
        std::cout << q.coeffs().transpose() << std::endl; // x, y, z, w
    }

    {
        Eigen::Vector4d q2(0.038135, 0.18931, 0.2393, 0.95155);
        Eigen::Matrix4d R = LeftQuatMatrix(q1);
        Eigen::Vector4d q = R * q2;
        std::cout << q.transpose() << std::endl; // x, y, z, w
    }
    return 0;
}
