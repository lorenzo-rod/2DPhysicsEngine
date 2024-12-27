#include "vector2.h"

namespace flatmath
{
    Vector2 Vector2::addVector(const Vector2 &other) const
    {
        return Vector2{this->x + other.x, this->y + other.y};
    }

    Vector2 Vector2::subtractVector(const Vector2 &other) const
    {
        return Vector2{this->x - other.x, this->y - other.y};
    }

    Vector2 Vector2::multiplyVector(float num) const
    {
        return Vector2{this->x * num, this->y * num};
    }

    bool Vector2::equals(const Vector2 &other) const
    {
        return this->x == other.x && this->y == other.y;
    }

    float Vector2::modulus() const
    {
        return pow(pow(this->x, 2.f) + pow(this->y, 2.f), 0.5f);
    }

    Vector2 Vector2::normalize() const
    {
        return *this * (1 / (this->modulus()));
    }

    float Vector2::dotProduct(const Vector2 &other) const
    {
        return this->x * other.x + this->y * other.y;
    }

    Vector2 operator+(const Vector2 &v1, const Vector2 &v2)
    {
        return v1.addVector(v2);
    }

    Vector2 operator-(const Vector2 &v1, const Vector2 &v2)
    {
        return v1.subtractVector(v2);
    }

    Vector2 operator-(const Vector2 &v)
    {
        return Vector2{-v.x, -v.y};
    }

    void Vector2::operator+=(const Vector2 &other)
    {
        this->x += other.x;
        this->y += other.y;
    }

    void Vector2::operator-=(const Vector2 &other)
    {
        this->x -= other.x;
        this->y -= other.y;
    }

    float crossProduct(const Vector2 &v1, const Vector2 &v2)
    {
        return v1.x * v2.y - v2.x * v1.y;
    }

    Vector2 operator*(const Vector2 &v, float num)
    {
        return v.multiplyVector(num);
    }

    Vector2 operator*(float num, const Vector2 &v)
    {
        return v * num;
    }

    float operator*(const Vector2 &v1, const Vector2 &v2)
    {
        return v1.dotProduct(v2);
    }

    bool operator==(const Vector2 &v1, const Vector2 &v2)
    {
        return v1.equals(v2);
    }

    std::ostream &operator<<(std::ostream &out, const Vector2 &c)
    {
        out << "(" << c.x << ", " << c.y << ")";
        return out;
    }
}