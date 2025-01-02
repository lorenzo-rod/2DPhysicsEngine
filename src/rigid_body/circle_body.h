#pragma once
#include "rigid_body.h"

class CircleBody : public RigidBody
{
    float m_radius;

    void loadShape(int scale) override;

public:
    CircleBody(float mass, float rotation, float rotational_velocity,
               float restitution, const flatmath::Vector2 &position,
               const flatmath::Vector2 &velocity,
               const flatmath::Vector2 &force,
               float radius, int scale);
    CircleBody(const CircleBody &other);
    float getRadius() const;
    float getShapeRadius() const;
    std::unique_ptr<RigidBody> cloneIntoPtr() const override;
};
