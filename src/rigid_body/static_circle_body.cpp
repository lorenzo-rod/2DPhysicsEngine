#include "static_circle_body.h"

StaticCircleBody::StaticCircleBody(float rotation,
                                   float restitution,
                                   const flatmath::Vector2 &position,
                                   float radius)
    : RigidBody(0.f, rotation, 0.f, restitution, position, {0.f, 0.f}, {0.f, 0.f}),
      CircleBody(radius),
      StaticBody()
{
}

StaticCircleBody::StaticCircleBody(const StaticCircleBody &other) : RigidBody(0.f,
                                                                              other.getRotation(),
                                                                              0.f,
                                                                              other.getRestitution(),
                                                                              other.getPosition(),
                                                                              {0.f, 0.f},
                                                                              {0.f, 0.f}),
                                                                    CircleBody(other.getRadius()),
                                                                    StaticBody()
{
}

std::unique_ptr<RigidBody> StaticCircleBody::cloneIntoPtr() const
{
    return std::make_unique<StaticCircleBody>(*this);
}
