#include "segment3d_utils.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <optional>

#include "common_utils.h"

/* Note: I left comments and debug prints here on purpose*/

namespace
{
    using utils::two_diff;
    using utils::two_product;
    using utils::two_sum;

    long double guard_product(long double first_sq, long double second_sq)
    {
        long double product = two_product(first_sq, second_sq);
        return (product < 1.0L) ? 1.0L : product;
    }

    long double guard_triple(long double a_sq, long double b_sq, long double c_sq)
    {
        long double ab = two_product(a_sq, b_sq);
        long double abc = two_product(ab, c_sq);
        return (abc < 1.0L) ? 1.0L : abc;
    }
} // namespace

namespace utils
{
    std::optional<IntersectionResult> intersect(const space_elements::Segment3D &seg1,
                                                const space_elements::Segment3D &seg2)
    {
        space_elements::Vector3D p1 = seg1.get_start();
        // space_elements::Vector3D p2 = seg1.get_end(); we actually do not need this variable, but one should understand the naming
        space_elements::Vector3D p3 = seg2.get_start();
        space_elements::Vector3D p4 = seg2.get_end();

        space_elements::Vector3D d1 = seg1.get_direction(); // seg1: p1 + t*d1
        space_elements::Vector3D d2 = seg2.get_direction(); // seg2: p3 + s*d2

        const long double epsilon = 1e-15L;
        const long double epsilon_sq = two_product(epsilon, epsilon);

        long double d1_len_sq = d1.length_squared();
        long double d2_len_sq = d2.length_squared();
        long double reference_length_sq = std::max({d1_len_sq,
                                                    d2_len_sq,
                                                    (p3 - p1).length_squared(),
                                                    (p4 - p1).length_squared(),
                                                    1.0L});

        bool seg1_is_point = (d1_len_sq < epsilon);
        bool seg2_is_point = (d2_len_sq < epsilon);

        long double inv_d1_len_sq = seg1_is_point ? 0.0L : (1.0L / d1_len_sq);

        // Case 1: Both segments are points
        if (seg1_is_point && seg2_is_point)
        {
            space_elements::Vector3D diff = p1 - p3;
            if (diff.length_squared() <= epsilon_sq)
            {
                std::cout << "Both segments are points at the same location." << std::endl;
                return IntersectionResult(p1);
            }
            else
            {
                std::cout << "Both segments are points at different locations." << std::endl;
                return std::nullopt;
            }
        }

        // Case 2: First segment is a point, second is not
        if (seg1_is_point)
        {
            std::cout << "First segment is a point." << std::endl;
            // Check if point p1 lies on segment seg2
            // p1 should equals p3 + s*d2 with one s in [0,1]
            space_elements::Vector3D w = p1 - p3;
            long double w_len_sq = w.length_squared();
            space_elements::Vector3D w_cross_d2 = w ^ d2;
            long double cross_scale = guard_product(d2_len_sq, w_len_sq);

            // Cross product should be zero because if p1 lies on the line of seg2 then w and d2 are collinear
            if (w_cross_d2.length_squared() > two_product(epsilon, cross_scale))
            {
                std::cout << "Point does not lie on the second segment." << std::endl;
                return std::nullopt;
            }

            long double dot = w * d2;
            long double range_tolerance = two_product(epsilon, std::max(d2_len_sq, reference_length_sq));
            long double dot_upper_bound = two_sum(d2_len_sq, range_tolerance);

            // Dot product should be in [0, d2_len_sq] range for some precision
            // Here I'm getting rid of the division(because division is usually bad for precision in floating point arithmetic)
            // So I compare dot with 0 and d2_len_sq directly with some tolerance instead of computing s = dot / d2_len_sq and checking if s in [0,1]
            if (dot < -range_tolerance || dot > dot_upper_bound)
            {
                std::cout << "Point does not lie on the second segment." << std::endl;
                return std::nullopt;
            }

            std::cout << "Point lies on the second segment." << std::endl;
            return IntersectionResult(p1);
        }

        // Case 3: Second segment is a point, first is not
        if (seg2_is_point)
        {
            std::cout << "Second segment is a point." << std::endl;
            // Check if point p3 lies on segment seg1
            space_elements::Vector3D w = p3 - p1;
            long double w_len_sq = w.length_squared();
            space_elements::Vector3D w_cross_d1 = w ^ d1;
            long double cross_scale = guard_product(d1_len_sq, w_len_sq);

            if (w_cross_d1.length_squared() > two_product(epsilon, cross_scale))
            {
                std::cout << "Point does not lie on the first segment." << std::endl;
                return std::nullopt;
            }

            long double dot = w * d1;
            long double range_tolerance = two_product(epsilon, std::max(d1_len_sq, reference_length_sq));
            long double dot_upper_bound = two_sum(d1_len_sq, range_tolerance);

            if (dot < -range_tolerance || dot > dot_upper_bound)
            {
                std::cout << "Point does not lie on the first segment." << std::endl;
                return std::nullopt;
            }

            std::cout << "Point lies on the first segment." << std::endl;
            return IntersectionResult(p3);
        }

        // Case 4: Both segments are actual segments (not points)
        // Check if 4 points are coplanar
        space_elements::Vector3D v1 = d1;
        space_elements::Vector3D v2 = p3 - p1;
        space_elements::Vector3D v3 = (p3 + d2) - p1;

        space_elements::Vector3D w = p3 - p1;
        long double w_len_sq = w.length_squared();
        space_elements::Vector3D d1_cross_d2 = d1 ^ d2;
        long double d1_cross_d2_length_sq = d1_cross_d2.length_squared();

        // Det of [v1 | v2 | v3] matrix (mixed product)
        long double det = v1 * (v2 ^ v3);

        long double v1_len_sq = v1.length_squared();
        long double v2_len_sq = v2.length_squared();
        long double v3_len_sq = v3.length_squared();
        long double det_scale_sq = guard_triple(v1_len_sq, v2_len_sq, v3_len_sq);
        long double det_sq = two_product(det, det);
        bool are_coplanar = (det_sq <= two_product(epsilon_sq, det_scale_sq));

        long double direction_pair_sq = guard_product(d1_len_sq, d2_len_sq);
        long double parallel_tolerance = two_product(epsilon, direction_pair_sq);
        // Vectors are parallel => cross product is zero (with some precision)
        bool are_parallel = (d1_cross_d2_length_sq <= parallel_tolerance);

        if (are_parallel)
        {
            std::cout << "Segments are parallel." << std::endl;

            // Parallel and not coplanar => no intersection
            if (!are_coplanar)
            {
                std::cout << "Segments are parallel but not coplanar - no intersection." << std::endl;
                return std::nullopt;
            }

            // Parallel and coplanar => check if they lie on the same line
            // We can check this by testing if w ^ d1 = 0 with some precision
            space_elements::Vector3D w_cross_d1 = w ^ d1;
            long double w_cross_d1_len_sq = w_cross_d1.length_squared();
            long double cross_scale = guard_product(d1_len_sq, w_len_sq);

            if (w_cross_d1_len_sq > two_product(epsilon, cross_scale))
            {
                std::cout << "Segments are parallel and coplanar but on different lines." << std::endl;
                return std::nullopt;
            }

            std::cout << "Segments are collinear (on the same line)." << std::endl;

            // Segments are on the same line - find overlapping region
            // d1_len_sq already computed at the beginning of the function

            // Parameters of points on the line (p1 is the origin)
            long double t1 = 0.0L;
            long double t2 = 1.0L;

            // Project p3 and p4 onto d1
            long double t3 = two_product((p3 - p1) * d1, inv_d1_len_sq);
            long double t4 = two_product((p4 - p1) * d1, inv_d1_len_sq);

            if (t3 > t4)
            {
                long double temp = t3;
                t3 = t4;
                t4 = temp;
            }

            // Find intersection of segments [t1, t2] and [t3, t4]
            long double t_start = (t1 > t3) ? t1 : t3;
            long double t_end = (t2 < t4) ? t2 : t4;

            if (t_start > two_sum(t_end, epsilon))
            {
                std::cout << "Collinear segments do not overlap." << std::endl;
                return std::nullopt;
            }

            if (two_diff(t_end, t_start) < epsilon)
            {
                space_elements::Vector3D intersection_point = p1 + (d1 * t_start);
                std::cout << "Collinear segments touch at a single point." << std::endl;
                return IntersectionResult(intersection_point);
            }
            else
            {
                space_elements::Vector3D overlap_start = p1 + (d1 * t_start);
                space_elements::Vector3D overlap_end = p1 + (d1 * t_end);
                space_elements::Segment3D overlap_segment(overlap_start, overlap_end);
                std::cout << "Collinear segments overlap in a segment." << std::endl;
                return IntersectionResult(overlap_segment);
            }
        }

        if (!are_coplanar)
        {
            std::cout << "Segments are skew (not coplanar) - no intersection." << std::endl;
            return std::nullopt;
        }

        // Segments are not parallel and coplanar - find intersection point
        std::cout << "Segments are coplanar and not parallel - checking intersection." << std::endl;

        // Calculate parameters t and s for points on the lines
        space_elements::Vector3D w_cross_d1 = w ^ d1;
        space_elements::Vector3D w_cross_d2 = w ^ d2;

        /*
        Some explanation here needed:
        Let p1 = A, p2 = B, p3 = C, p4 = D
        Then AB = p2 - p1, CD = p4 - p3
        With X = intersection point and O = origin of coordinates
        Lets write down some equations:
        OX = OA + t*AB
        OX = OC + s*CD
        OA + t*AB = OC + s*CD
        Rearranging:
        t*AB - s*CD = OC - OA
        t*AB - s*CD = AC
        In our terms:
        AB = d1, CD = d2, AC = w
        t*d1 - s*d2 = w
        Finally we can find t and s by cross-multiplying both sides with d2 and d1 respectively:
        t*(d1 ^ d2) -s*(d2 ^ d2) = w ^ d2
        With d2 ^ d2 = 0 we have:
        t*(d1 ^ d2) = w ^ d2
        t = ((w ^ d2) * (d1 ^ d2)) / |d1 ^ d2|^2
        Similarly for s:
        t*(d1 ^ d1) - s*(d2 ^ d1) = w ^ d1
        - s*(d2 ^ d1) = w ^ d1
        s*(d1 ^ d2) = w ^ d1
        s = ((w ^ d1) * (d1 ^ d2)) / |d1 ^ d2|^2
        With |d1 ^ d2|^2 = squared length of cross product of d1 and d2 we have equations for t and s below:
        */
        long double inv_cross_len_sq = 1.0L / d1_cross_d2_length_sq;
        long double t_numerator = (w_cross_d2 * d1_cross_d2);
        long double s_numerator = (w_cross_d1 * d1_cross_d2);
        long double t = two_product(t_numerator, inv_cross_len_sq);
        long double s = two_product(s_numerator, inv_cross_len_sq);

        // If t or s are outside [0, 1] range, the intersection point is outside the segments
        long double param_tolerance = two_product(epsilon, 10.0L);
        long double param_upper_bound = two_sum(1.0L, param_tolerance);
        if (t < -param_tolerance || t > param_upper_bound || s < -param_tolerance || s > param_upper_bound)
        {
            std::cout << "Intersection point is outside one or both segments." << std::endl;
            return std::nullopt;
        }

        long double t_inbound = std::min(std::max(t, 0.0L), 1.0L);
        long double s_inbound = std::min(std::max(s, 0.0L), 1.0L);

        space_elements::Vector3D point1_x = p1 + (d1 * t_inbound);
        space_elements::Vector3D point2_x = p3 + (d2 * s_inbound);

        // Check that the points are the same (segments actually intersect in 3D)
        space_elements::Vector3D diff = point1_x - point2_x;
        long double distance_sq = diff.length_squared();

        std::cout << "Distance squared between intersection points: " << distance_sq << std::endl;

        if (distance_sq <= epsilon_sq)
        {
            std::cout << "Segments intersect at a point." << std::endl;
            return IntersectionResult(point1_x);
        }

        return std::nullopt;
    }
} // namespace utils
