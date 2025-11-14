#include "vector3d.h"

#include <cmath>

#include "common_utils.h"

namespace space_elements
{
    Vector3D &Vector3D::operator+=(const Vector3D &other)
    {
        _X = utils::two_sum(_X, other._X);
        _Y = utils::two_sum(_Y, other._Y);
        _Z = utils::two_sum(_Z, other._Z);
        _length_squared = calculate_length_squared();
        _length = sqrtl(_length_squared);
        return *this;
    }

    Vector3D &Vector3D::operator-=(const Vector3D &other)
    {
        _X = utils::two_diff(_X, other._X);
        _Y = utils::two_diff(_Y, other._Y);
        _Z = utils::two_diff(_Z, other._Z);
        _length_squared = calculate_length_squared();
        _length = sqrtl(_length_squared);
        return *this;
    }

    Vector3D Vector3D::operator+(const Vector3D &other) const
    {
        Vector3D result(*this);
        result += other;
        return result;
    }

    Vector3D Vector3D::operator-(const Vector3D &other) const
    {
        Vector3D result(*this);
        result -= other;
        return result;
    }

    long double Vector3D::operator*(const Vector3D &other) const
    {
        long double p1 = utils::two_product(_X, other._X);
        long double p2 = utils::two_product(_Y, other._Y);
        long double p3 = utils::two_product(_Z, other._Z);
        long double sum1 = utils::two_sum(p1, p2);
        return utils::two_sum(sum1, p3);
    }

    Vector3D Vector3D::operator^(const Vector3D &other) const
    {
        long double p1 = utils::two_product(_Y, other._Z);
        long double p2 = utils::two_product(_Z, other._Y);
        long double x = utils::two_diff(p1, p2);

        long double p3 = utils::two_product(_Z, other._X);
        long double p4 = utils::two_product(_X, other._Z);
        long double y = utils::two_diff(p3, p4);

        long double p5 = utils::two_product(_X, other._Y);
        long double p6 = utils::two_product(_Y, other._X);
        long double z = utils::two_diff(p5, p6);

        return Vector3D(x, y, z);
    }

    Vector3D Vector3D::operator*(long double scalar) const
    {
        long double x = utils::two_product(_X, scalar);
        long double y = utils::two_product(_Y, scalar);
        long double z = utils::two_product(_Z, scalar);
        return Vector3D(x, y, z);
    }

    long double Vector3D::calculate_length_squared() const
    {
        long double x2 = utils::two_product(_X, _X);
        long double y2 = utils::two_product(_Y, _Y);
        long double z2 = utils::two_product(_Z, _Z);
        long double sum1 = utils::two_sum(x2, y2);
        return utils::two_sum(sum1, z2);
    }

    long double Vector3D::calculate_length() const
    {
        return sqrtl(_length_squared);
    }
} // namespace space_elements