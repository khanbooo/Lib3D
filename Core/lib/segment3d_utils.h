#ifndef CORE_LIB_SEGMENT3D_UTILS_H
#define CORE_LIB_SEGMENT3D_UTILS_H

#include <optional>
#include <variant>

#include "segment3d.h"
#include "vector3d.h"

namespace utils
{
    using IntersectionResult = std::variant<space_elements::Vector3D, space_elements::Segment3D>;

    std::optional<IntersectionResult> intersect(const space_elements::Segment3D &seg1,
                                                const space_elements::Segment3D &seg2);
} // namespace utils

#endif // CORE_LIB_SEGMENT3D_UTILS_H
