#include "point2.h"

namespace flatmath
{
    Vector2 genVec(const Point2 &p1, const Point2 &p2)
    {
        return Vector2{p1.x - p2.x, p1.y - p2.y};
    }

    float distance(const Point2 &p1, const Point2 &p2)
    {
        return (p2 - p1).modulus();
    }

    Vector2 operator-(const Point2 &p1, const Point2 &p2)
    {
        return genVec(p1, p2);
    }
}
