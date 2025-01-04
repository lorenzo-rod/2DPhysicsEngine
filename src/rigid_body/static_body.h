#pragma once
#include "rigid_body.h"

class StaticBody : virtual public RigidBody
{
public:
    StaticBody();
    StaticBody(float rotation, float restitution,
               const flatmath::Vector2 &position);
    void setVelocity(const flatmath::Vector2 &velocity) override;
    void move(const flatmath::Vector2 &vec) override;
};
