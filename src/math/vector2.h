#pragma once
#include <cmath>
#include <iostream>

namespace flatmath
{

    struct Vector2
    {
        float x;
        float y;

        Vector2 addVector(const Vector2 &other) const;
        Vector2 subtractVector(const Vector2 &other) const;
        Vector2 multiplyVector(float num) const;
        bool equals(const Vector2 &other) const;

        float modulus() const;
        Vector2 normalize() const;
        float dotProduct(const Vector2 &other) const;

        void operator+=(const Vector2 &other);
        void operator-=(const Vector2 &other);
    };

    float crossProduct(const Vector2 &v1, const Vector2 &v2);
    Vector2 operator+(const Vector2 &v1, const Vector2 &v2);
    Vector2 operator-(const Vector2 &v1, const Vector2 &v2);
    Vector2 operator-(const Vector2 &v);
    Vector2 operator*(const Vector2 &v, float num);
    Vector2 operator*(float num, const Vector2 &v);
    float operator*(const Vector2 &v1, const Vector2 &v2);
    bool operator==(const Vector2 &v1, const Vector2 &v2);
    std::ostream &operator<<(std::ostream &out, const Vector2 &c);

}