#pragma once
#include "circle_body.h"
#include "static_body.h"

class StaticCircleBody : public CircleBody, public StaticBody
{
    float m_radius;

public:
    StaticCircleBody(float rotation, float restitution,
                     const flatmath::Vector2 &position,
                     float radius);
    StaticCircleBody(const StaticCircleBody &other);
    std::unique_ptr<RigidBody> cloneIntoPtr() const override;
};