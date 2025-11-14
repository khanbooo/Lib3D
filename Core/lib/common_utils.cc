#include "common_utils.h"

#include <cmath>

namespace utils
{
    long double two_sum(long double a, long double b)
    {
        long double s = a + b;
        long double a_virtual = s - b;
        long double b_virtual = s - a_virtual;
        long double a_roundoff = a - a_virtual;
        long double b_roundoff = b - b_virtual;
        long double error = a_roundoff + b_roundoff;
        return s + error;
    }

    long double two_diff(long double a, long double b)
    {
        long double d = a - b;
        long double a_virtual = d + b;
        long double b_virtual = a_virtual - d;
        long double a_roundoff = a - a_virtual;
        long double b_roundoff = b_virtual - b;
        long double error = a_roundoff + b_roundoff;
        return d + error;
    }

    long double two_product(long double a, long double b)
    {
        long double p = a * b;
        long double error = std::fmal(a, b, -p);
        return p + error;
    }
} // namespace utils