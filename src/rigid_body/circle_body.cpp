#include "circle_body.h"

CircleBody::CircleBody(float radius) : RigidBody(0.f, 0.f, 0.f, 0.f, {0.f, 0.f}, {0.f, 0.f}, {0.f, 0.f}),
                                       m_radius(radius)
{
    loadShape();
}

CircleBody::CircleBody(float inv_mass, float rotation,
                       float rotational_velocity,
                       float restitution,
                       const flatmath::Vector2 &position,
                       const flatmath::Vector2 &velocity,
                       const flatmath::Vector2 &force,
                       float radius)
    : RigidBody(inv_mass, rotation, rotational_velocity, restitution, position, velocity, force), m_radius(radius)
{
    loadShape();
}

CircleBody::CircleBody(const CircleBody &other) : RigidBody(other.getInvMass(),
                                                            other.getRotation(),
                                                            other.getRotationalVelocity(),
                                                            other.getRestitution(),
                                                            other.getPosition(),
                                                            other.getVelocity(),
                                                            other.getForce()),
                                                  m_radius(other.getRadius())
{
    loadShape();
}

float CircleBody::getRadius() const
{
    return m_radius;
}

void CircleBody::loadShape()
{
    m_shape_ptr = std::make_unique<sf::CircleShape>(m_radius);
    m_shape_ptr->setOrigin(m_radius, m_radius);
}

std::unique_ptr<RigidBody> CircleBody::cloneIntoPtr() const
{
    return std::make_unique<CircleBody>(*this);
}
