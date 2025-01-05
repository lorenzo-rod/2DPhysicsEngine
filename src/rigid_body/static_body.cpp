#include "static_body.h"

StaticBody::StaticBody()
    : RigidBody(0.f, 0.f, 0.f, 0.f, {0.f, 0.f}, {0.f, 0.f}, {0.f, 0.f})
{
}

StaticBody::StaticBody(float rotation, float restitution,
                       const flatmath::Vector2 &position)
    : RigidBody(0.f, rotation, 0.f, restitution, position, {0.f, 0.f}, {0.f, 0.f})
{
}

void StaticBody::setVelocity(const flatmath::Vector2 &velocity)
{
}

void StaticBody::addVelocity(const flatmath::Vector2 &velocity)
{
}

void StaticBody::addForce(const flatmath::Vector2 &force)
{
}

void StaticBody::move(const flatmath::Vector2 &vec)
{
}
