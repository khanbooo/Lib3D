#ifndef CORE_LIB_SEGMENT3D_H
#define CORE_LIB_SEGMENT3D_H

#include <cmath>
#include <utility>

#include "vector3d.h"

namespace space_elements
{
    class Segment3D
    {
    public:
        Segment3D(const Vector3D &Start, const Vector3D &End)
            : _start(Start), _end(End), _direction(End - Start) {}
        Segment3D()
            : _start(Vector3D()), _end(Vector3D()), _direction(Vector3D()) {}
        Segment3D(const Segment3D &other)
            : _start(other._start), _end(other._end), _direction(other._direction) {}
        Segment3D(Segment3D &&other) noexcept
            : _start(std::move(other._start)), _end(std::move(other._end)),
              _direction(std::move(other._direction)) {}

        Segment3D &operator+=(const Vector3D &vec);
        Segment3D &operator-=(const Vector3D &vec);
        Segment3D operator+(const Vector3D &vec);
        Segment3D operator-(const Vector3D &vec);

        Vector3D get_start() const { return _start; }
        Vector3D get_end() const { return _end; }
        Vector3D get_direction() const { return _direction; }
        long double length() const { return _direction.length(); }
        long double length_squared() const { return _direction.length_squared(); }

    private:
        Vector3D _start;
        Vector3D _end;
        Vector3D _direction;
    };
} // namespace space_elements

#endif // CORE_LIB_SEGMENT3D_H