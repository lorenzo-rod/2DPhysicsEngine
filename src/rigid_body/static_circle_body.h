#pragma once
#include "circle_body.h"

class StaticCircleBody : public CircleBody
{
    float m_radius;

public:
    StaticCircleBody(float rotation, float restitution,
                     const flatmath::Vector2 &position,
                     const flatmath::Vector2 &force,
                     float radius);
    StaticCircleBody(const StaticCircleBody &other);
    void move(const flatmath::Vector2 &vec) override;
    void setVelocity(const flatmath::Vector2 &velocity) override;
    std::unique_ptr<RigidBody> cloneIntoPtr() const override;
};