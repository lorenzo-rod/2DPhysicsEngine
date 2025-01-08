#pragma once
#include "rigid_body.h"

class StaticBody : virtual public RigidBody
{
public:
    StaticBody();
    StaticBody(float rotation, float restitution,
               const flatmath::Vector2 &position);
    void setVelocity(const flatmath::Vector2 &velocity) override;
    void addVelocity(const flatmath::Vector2 &velocity) override;
    void setRotationalVelocity(float rot_vel) override;
    void addRotationalVelocity(float rot_vel) override;
    void addForce(const flatmath::Vector2 &force) override;
    void move(const flatmath::Vector2 &vec) override;
};
