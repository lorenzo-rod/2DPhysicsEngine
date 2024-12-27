#pragma once
#include "rigid_body.h"

class CircleBody : public RigidBody
{
    float m_radius;

public:
    CircleBody(float mass, float rotation, const flatmath::Vector2 &position,
               const flatmath::Vector2 &velocity, const flatmath::Vector2 &force, float radius, int scale);
    void loadShape(int scale) override;
};
