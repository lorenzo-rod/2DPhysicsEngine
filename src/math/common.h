#pragma once
#include "vector2.h"

namespace flatmath
{
    float pointToSegmentSquaredDistance(const flatmath::Vector2 &a,
                                        const flatmath::Vector2 &b,
                                        const flatmath::Vector2 &point,
                                        flatmath::Vector2 &contact_point);
    float distance(const flatmath::Vector2 &a, const flatmath::Vector2 &b);
    float squaredDistance(const flatmath::Vector2 &a, const flatmath::Vector2 &b);
}
