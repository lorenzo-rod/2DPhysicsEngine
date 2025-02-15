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

    bool Vector2::approximatelyEquals(const Vector2 &other, float epsilon)
    {
        return (flatmath::approximatelyEquals(this->x, other.x, epsilon) &&
                flatmath::approximatelyEquals(this->y, other.y, epsilon));
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

    float Vector2::crossProduct(const Vector2 &other)
    {
        return this->x * other.y - other.x * this->y;
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