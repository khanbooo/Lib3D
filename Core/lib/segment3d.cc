#include "segment3d.h"

#include <cmath>

#include "segment3d_utils.h"

namespace space_elements
{
    Segment3D &Segment3D::operator+=(const Vector3D &vec)
    {
        _start += vec;
        _end += vec;
        return *this;
    }

    Segment3D &Segment3D::operator-=(const Vector3D &vec)
    {
        _start -= vec;
        _end -= vec;
        return *this;
    }

    Segment3D Segment3D::operator+(const Vector3D &vec)
    {
        Segment3D result(*this);
        result += vec;
        return result;
    }

    Segment3D Segment3D::operator-(const Vector3D &vec)
    {
        Segment3D result(*this);
        result -= vec;
        return result;
    }

} // namespace space_elements