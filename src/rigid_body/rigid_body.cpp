#include "rigid_body.h"

RigidBody::RigidBody(float mass, float rotation,
                     const flatmath::Vector2 &position,
                     const flatmath::Vector2 &velocity,
                     const flatmath::Vector2 &force)
    : m_mass(mass),
      m_rotation(rotation),
      m_position(position),
      m_velocity(velocity),
      m_force(force)
{
}
float RigidBody::getMass() const
{
    return m_mass;
}
float RigidBody::getRotation() const
{
    return m_rotation;
}
flatmath::Vector2 RigidBody::getPosition() const
{
    return m_position;
}
flatmath::Vector2 RigidBody::getVelocity() const
{
    return m_velocity;
}
flatmath::Vector2 RigidBody::getForce() const
{
    return m_force;
}
void RigidBody::step(float dt)
{
    m_velocity += m_force * (1 / m_mass) * dt;
    m_position += m_velocity * dt;
}
void RigidBody::draw(sf::RenderWindow &window)
{
    sf::Transform shape_transform;
    shape_transform.translate(m_position.x, m_position.y);
    shape_transform.rotate(m_rotation);
    window.draw(*m_shape_ptr, shape_transform);
}
