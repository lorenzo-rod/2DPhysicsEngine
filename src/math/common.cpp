#include "common.h"

namespace flatmath
{
    float pointToSegmentSquaredDistance(const flatmath::Vector2 &a,
                                        const flatmath::Vector2 &b,
                                        const flatmath::Vector2 &point,
                                        flatmath::Vector2 &contact_point)
    {
        Vector2 ab = b - a;
        Vector2 ap = point - a;

        float projection = ap * ab;
        float ab_squared_len = squaredDistance(a, b);
        float d = projection / ab_squared_len;

        if (d <= 0.f)
        {
            contact_point = a;
        }
        else if (d >= 1.f)
        {
            contact_point = b;
        }
        else
        {
            contact_point = a + ab * d;
        }

        return squaredDistance(contact_point, point);
    }

    float distance(const flatmath::Vector2 &a, const flatmath::Vector2 &b)
    {
        return (a - b).modulus();
    }

    float squaredDistance(const flatmath::Vector2 &a, const flatmath::Vector2 &b)
    {
        return pow((a.x - b.x), 2) + pow((a.y - b.y), 2);
    }
}
