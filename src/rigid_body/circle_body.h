#pragma once
#include "rigid_body.h"

class CircleBody : public RigidBody
{
    float m_radius;

    void loadShape() override;

public:
    CircleBody(float mass, float rotation, float rotational_velocity,
               float restitution, const flatmath::Vector2 &position,
               const flatmath::Vector2 &velocity,
               const flatmath::Vector2 &force,
               float radius);
    CircleBody(const CircleBody &other);
    float getRadius() const;
    std::unique_ptr<RigidBody> cloneIntoPtr() const override;
};
