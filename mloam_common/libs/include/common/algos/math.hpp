#ifndef _MATH_HPP_
#define _MATH_HPP_

#include <cmath>

namespace common
{
    // -----------------
    ///< Scalar calculation
    template <typename T>
    inline T sqrSum(const T x, const T y, const T z)
    {
        return T(x * x + y * y + z * z);
    }

    template <typename T>
    static T toRad(T degree)
    {
        return T(degree * M_PI / 180.f);
    }

    template <typename T>
    static T toDeg(T rad)
    {
        return T(rad / M_PI * 180.f);
    }

    template <typename T>
    bool solveQuadraticEquation(const T a, const T b, const T c, T &x1, T &x2)
    {
        if (fabs(a) < 1e-12)
        {
            x1 = x2 = -c / b;
            return true;
        }
        T delta2 = b * b - 4.0 * a * c;
        if (delta2 < 0.0)
        {
            return false;
        }
        T delta = sqrt(delta2);
        x1 = (-b + delta) / (2.0 * a);
        x2 = (-b - delta) / (2.0 * a);
        return true;
    }

    // -----------------
    ///< Matrix calculation
    static const Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

    template<typename Derived>
    inline Eigen::Quaternion<typename Derived::Scalar> DeltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    template<typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &v3d)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
        m << typename Derived::Scalar(0), -v3d.z(), v3d.y(),
            v3d.z(), typename Derived::Scalar(0), -v3d.x(),
            -v3d.y(), v3d.x(), typename Derived::Scalar(0);
        return m;
    }

    template<typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
        Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
        typename Derived::Scalar q4 = q.w();
        m.block(0, 0, 3, 3) << q4 * I3x3 + SkewSymmetric(vq);
        m.block(3, 0, 1, 3) << -vq.transpose();
        m.block(0, 3, 3, 1) << vq;
        m(3, 3) = q4;
        return m;
    }

    template<typename Derived>
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

    template<typename T>
    inline Eigen::Matrix<T, 4, 4> LeftQuatMatrix(const Eigen::Matrix<T, 4, 1> &q)
    {
        Eigen::Matrix<T, 4, 4> m;
        Eigen::Matrix<T, 3, 1> vq{q.x(), q.y(), q.z()};
        T q4 = q.w();
        m.block(0, 0, 3, 3) << q4 * I3x3 + SkewSymmetric(vq);
        m.block(3, 0, 1, 3) << -vq.transpose();
        m.block(0, 3, 3, 1) << vq;
        m(3, 3) = q4;
        return m;
    }

    template<typename T>
    inline Eigen::Matrix<T, 4, 4> RightQuatMatrix(const Eigen::Matrix<T, 4, 1> &p)
    {
        Eigen::Matrix<T, 4, 4> m;
        Eigen::Matrix<T, 3, 1> vp{p.x(), p.y(), p.z()};
        T p4 = p.w();
        m.block(0, 0, 3, 3) << p4 * I3x3 - SkewSymmetric(vp);
        m.block(3, 0, 1, 3) << -vp.transpose();
        m.block(0, 3, 3, 1) << vp;
        m(3, 3) = p4;
        return m;
    }

    inline Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    template <typename MatrixType>
    inline typename MatrixType::Scalar logDet(const MatrixType &M, bool use_cholesky = false)
    {
        using namespace Eigen;
        typedef typename MatrixType::Scalar Scalar;
        
        Scalar ld = 0;
        if (use_cholesky)
        {
            LLT<Matrix<Scalar, Dynamic, Dynamic>> chol(M);
            auto &U = chol.matrixL();
            for (unsigned i = 0; i < M.rows(); ++i)
                ld += std::log(U(i, i)); // or ld+= std::log(prod(U.diagonal()))
            ld *= 2;
        }
        else
        {
            PartialPivLU<Matrix<Scalar, Dynamic, Dynamic> > lu(M);
            auto &LU = lu.matrixLU();
            Scalar c = lu.permutationP().determinant(); // -1 or 1
            for (unsigned i = 0; i < LU.rows(); ++i)
            {
                const auto &lii = LU(i, i);
                if (lii < Scalar(0))
                    c *= -1;
                ld += std::log(abs(lii));
            }
            ld += std::log(c);
        }
        return ld;
    }
}

#endif
