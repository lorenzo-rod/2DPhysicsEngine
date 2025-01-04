#include "static_rectangle_body.h"

StaticRectangleBody::StaticRectangleBody(float rotation,
                                         float restitution,
                                         const flatmath::Vector2 &position,
                                         float length, float height)
    : RigidBody(0.f, rotation, 0.f, restitution, position, {0.f, 0.f}, {0.f, 0.f}),
      RectangleBody(length, height),
      StaticBody()
{
}

StaticRectangleBody::StaticRectangleBody(const StaticRectangleBody &other) : RigidBody(0.f,
                                                                                       other.getRotation(),
                                                                                       0.f,
                                                                                       other.getRestitution(),
                                                                                       other.getPosition(),
                                                                                       {0.f, 0.f},
                                                                                       {0.f, 0.f}),
                                                                             RectangleBody(other.getLength(),
                                                                                           other.getHeight()),
                                                                             StaticBody()
{
}

std::unique_ptr<RigidBody> StaticRectangleBody::cloneIntoPtr() const
{
    return std::make_unique<StaticRectangleBody>(*this);
}
