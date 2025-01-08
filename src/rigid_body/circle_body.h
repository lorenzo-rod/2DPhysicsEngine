#pragma once
#include "rigid_body.h"

class CircleBody : virtual public RigidBody
{
protected:
    float m_radius;

    void loadShape() override;
    void loadInertiaMoment() override;

public:
    CircleBody(float radius);
    CircleBody(float inv_mass, float rotation, float rotational_velocity,
               float restitution, const flatmath::Vector2 &position,
               const flatmath::Vector2 &velocity,
               const flatmath::Vector2 &force,
               float torque,
               float radius);
    CircleBody(const CircleBody &other);
    float getRadius() const;
    std::unique_ptr<RigidBody> cloneIntoPtr() const override;
};
