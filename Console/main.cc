#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

#include "../Core/lib/vector3d.h"
#include "../Core/lib/segment3d.h"
#include "../Core/lib/segment3d_utils.h"

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <numbers...>" << std::endl;
        std::cerr << "Example: " << argv[0] << " 1.0 2.0 3.0 4.0 5.0 6.0" << std::endl;
        return 1;
    }

    std::vector<long double> numbers;
    for (int i = 1; i < argc; ++i)
    {
        try
        {
            long double value = std::stold(argv[i]);
            numbers.push_back(value);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error parsing argument " << i << ": " << argv[i] << std::endl;
            std::cerr << "Exception: " << e.what() << std::endl;
            return 1;
        }
    }

    std::cout << "Received " << numbers.size() << " numbers:" << std::endl;
    std::cout << std::setprecision(15);

    for (size_t i = 0; i < numbers.size(); ++i)
    {
        std::cout << "  [" << i << "] = " << numbers[i] << std::endl;
    }

    if (numbers.size() >= 12)
    {
        std::cout << "\n=== Creating two segments ===" << std::endl;

        space_elements::Vector3D p1_start(numbers[0], numbers[1], numbers[2]);
        space_elements::Vector3D p1_end(numbers[3], numbers[4], numbers[5]);
        space_elements::Segment3D seg1(p1_start, p1_end);

        space_elements::Vector3D p2_start(numbers[6], numbers[7], numbers[8]);
        space_elements::Vector3D p2_end(numbers[9], numbers[10], numbers[11]);
        space_elements::Segment3D seg2(p2_start, p2_end);

        std::cout << "Segment 1:" << std::endl;
        std::cout << "  Start: (" << p1_start.X() << ", " << p1_start.Y() << ", " << p1_start.Z() << ")" << std::endl;
        std::cout << "  End:   (" << p1_end.X() << ", " << p1_end.Y() << ", " << p1_end.Z() << ")" << std::endl;
        std::cout << "  Length: " << seg1.length() << std::endl;

        std::cout << "\nSegment 2:" << std::endl;
        std::cout << "  Start: (" << p2_start.X() << ", " << p2_start.Y() << ", " << p2_start.Z() << ")" << std::endl;
        std::cout << "  End:   (" << p2_end.X() << ", " << p2_end.Y() << ", " << p2_end.Z() << ")" << std::endl;
        std::cout << "  Length: " << seg2.length() << std::endl;

        auto intersection = utils::intersect(seg1, seg2);

        std::cout << "\n=== Intersection result ===" << std::endl;
        if (intersection.has_value())
        {
            if (std::holds_alternative<space_elements::Vector3D>(intersection.value()))
            {
                space_elements::Vector3D point = std::get<space_elements::Vector3D>(intersection.value());
                std::cout << "Segments intersect at a point: ("
                          << point.X() << ", "
                          << point.Y() << ", "
                          << point.Z() << ")" << std::endl;
            }
            else if (std::holds_alternative<space_elements::Segment3D>(intersection.value()))
            {
                space_elements::Segment3D overlap = std::get<space_elements::Segment3D>(intersection.value());
                std::cout << "Segments overlap in a segment:" << std::endl;
                std::cout << "  Overlap start: ("
                          << overlap.get_start().X() << ", "
                          << overlap.get_start().Y() << ", "
                          << overlap.get_start().Z() << ")" << std::endl;
                std::cout << "  Overlap end:   ("
                          << overlap.get_end().X() << ", "
                          << overlap.get_end().Y() << ", "
                          << overlap.get_end().Z() << ")" << std::endl;
                std::cout << "  Overlap length: " << overlap.length() << std::endl;
            }
        }
        else
        {
            std::cout << "Segments do not intersect (parallel, skew, or outside bounds)" << std::endl;
        }
    }
    else
    {
        std::cout << "\nNote: Provide at least 12 numbers to test segment intersection" << std::endl;
    }

    return 0;
}
