#ifndef CORE_LIB_VECTOR3D_H
#define CORE_LIB_VECTOR3D_H

#include <cmath>

namespace space_elements
{
    class Vector3D
    {
    public:
        Vector3D(long double X = 0.0L, long double Y = 0.0L, long double Z = 0.0L)
            : _X(X), _Y(Y), _Z(Z), _length_squared(calculate_length_squared()), _length(sqrtl(_length_squared)) {};

        Vector3D &operator+=(const Vector3D &other);
        Vector3D &operator-=(const Vector3D &other);
        Vector3D operator+(const Vector3D &other) const;
        Vector3D operator-(const Vector3D &other) const;
        // Dot product
        long double operator*(const Vector3D &other) const;
        // Cross product
        Vector3D operator^(const Vector3D &other) const;
        // Scalar multiplication
        Vector3D operator*(long double scalar) const;

        long double X() const { return _X; }
        long double Y() const { return _Y; }
        long double Z() const { return _Z; }
        long double length() const { return _length; }
        long double length_squared() const { return _length_squared; }

    private:
        long double calculate_length_squared() const;
        long double calculate_length() const;

        long double _X;
        long double _Y;
        long double _Z;
        long double _length_squared;
        long double _length;
    };
}

#endif // CORE_LIB_VECTOR3D_H
