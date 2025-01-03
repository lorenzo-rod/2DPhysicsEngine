#pragma once
#include "rectangle_body.h"

class StaticRectangleBody : public RectangleBody
{
    float m_length;
    float m_height;

public:
    StaticRectangleBody(float rotation, float restitution,
                        const flatmath::Vector2 &position,
                        const flatmath::Vector2 &force,
                        float length,
                        float height);
    StaticRectangleBody(const StaticRectangleBody &other);
    void move(const flatmath::Vector2 &vec) override;
    void setVelocity(const flatmath::Vector2 &velocity) override;
    std::unique_ptr<RigidBody> cloneIntoPtr() const override;
};
