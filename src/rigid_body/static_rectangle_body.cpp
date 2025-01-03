#include "static_rectangle_body.h"

StaticRectangleBody::StaticRectangleBody(float rotation,
                                         float restitution,
                                         const flatmath::Vector2 &position,
                                         const flatmath::Vector2 &force,
                                         float length, float height)
    : RectangleBody(0.f, rotation, 0.f, restitution, position, {0.f, 0.f}, force, length, height)
{
    loadShape();
}

StaticRectangleBody::StaticRectangleBody(const StaticRectangleBody &other) : RectangleBody(0.f,
                                                                                           other.getRotation(),
                                                                                           0.f,
                                                                                           other.getRestitution(),
                                                                                           other.getPosition(),
                                                                                           {0.f, 0.f},
                                                                                           other.getForce(),
                                                                                           other.getLength(),
                                                                                           other.getHeight())
{
    loadShape();
}

void StaticRectangleBody::move(const flatmath::Vector2 &vec)
{
}

void StaticRectangleBody::setVelocity(const flatmath::Vector2 &velocity)
{
}

std::unique_ptr<RigidBody> StaticRectangleBody::cloneIntoPtr() const
{
    return std::make_unique<StaticRectangleBody>(*this);
}
