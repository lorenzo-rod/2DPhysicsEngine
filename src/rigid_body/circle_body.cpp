#include "circle_body.h"

CircleBody::CircleBody(float mass, float rotation,
                       float rotational_velocity,
                       const flatmath::Vector2 &position,
                       const flatmath::Vector2 &velocity,
                       const flatmath::Vector2 &force,
                       float radius, int scale)
    : RigidBody(mass, rotation, rotational_velocity, position, velocity, force, scale), m_radius(radius)
{
    loadShape(m_scale);
}

CircleBody::CircleBody(const CircleBody &other) : RigidBody(other.getMass(),
                                                            other.getRotation(),
                                                            other.getRotationalVelocity(),
                                                            other.getPosition(),
                                                            other.getVelocity(),
                                                            other.getForce(),
                                                            other.getScale()),
                                                  m_radius(other.getRadius())
{
    loadShape(m_scale);
}

float CircleBody::getRadius() const
{
    return m_radius;
}

float CircleBody::getShapeRadius() const
{
    return m_radius * m_scale;
}

void CircleBody::loadShape(int scale)
{
    float shape_radius = m_radius * scale;
    m_shape_ptr = std::make_unique<sf::CircleShape>(shape_radius);
    m_shape_ptr->setOrigin(shape_radius, shape_radius);
}

std::unique_ptr<RigidBody> CircleBody::cloneIntoPtr() const
{
    return std::make_unique<CircleBody>(*this);
}
