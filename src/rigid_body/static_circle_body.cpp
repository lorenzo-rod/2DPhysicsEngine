#include "static_circle_body.h"

StaticCircleBody::StaticCircleBody(float rotation,
                                   float restitution,
                                   const flatmath::Vector2 &position,
                                   const flatmath::Vector2 &force,
                                   float radius)
    : CircleBody(0.f, rotation, 0.f, restitution, position, {0.f, 0.f}, force, radius)
{
    loadShape();
}

StaticCircleBody::StaticCircleBody(const StaticCircleBody &other) : CircleBody(0.f,
                                                                               other.getRotation(),
                                                                               0.f,
                                                                               other.getRestitution(),
                                                                               other.getPosition(),
                                                                               {0.f, 0.f},
                                                                               other.getForce(),
                                                                               other.getRadius())
{
    loadShape();
}

void StaticCircleBody::move(const flatmath::Vector2 &vec)
{
}

void StaticCircleBody::setVelocity(const flatmath::Vector2 &velocity)
{
}

std::unique_ptr<RigidBody> StaticCircleBody::cloneIntoPtr() const
{
    return std::make_unique<StaticCircleBody>(*this);
}
