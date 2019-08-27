#ifndef _MATH_HPP_
#define _MATH_HPP_

#include <cmath>

namespace common
{
    template <typename T>
    T sqrSum(const T x, const T y, const T z)
    {
        T sqr_sum = x*x + y*y + z*z;
        return sqr_sum;
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
}

#endif
