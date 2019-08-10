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
}

#endif
