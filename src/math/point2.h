#pragma once
#include <iostream>
#include "vector2.h"

namespace flatmath
{
    struct Point2
    {
        float x;
        float y;
    };

    Vector2 genVec(const Point2& p1, const Point2& p2);
    float distance(const Point2& p1, const Point2& p2);
    Vector2 operator-(const Point2& p1, const Point2& p2);
}
