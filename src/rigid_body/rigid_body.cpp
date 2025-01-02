#include "rigid_body.h"

RigidBody::RigidBody(float mass, float rotation,
                     float rotational_velocity,
                     float restitution,
                     const flatmath::Vector2 &position,
                     const flatmath::Vector2 &velocity,
                     const flatmath::Vector2 &force)
    : m_mass(mass),
      m_rotation(rotation),
      m_rotational_velocity(rotational_velocity),
      m_restitution(restitution),
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
float RigidBody::getRotationalVelocity() const
{
    return m_rotational_velocity;
}
float RigidBody::getRestitution() const
{
    return m_restitution;
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
void RigidBody::addForce(const flatmath::Vector2 &force)
{
    m_force += force;
}
void RigidBody::setVelocity(const flatmath::Vector2 &velocity)
{
    m_velocity = velocity;
}
void RigidBody::move(const flatmath::Vector2 &vec)
{
    m_position += vec;
}
void RigidBody::step(float dt)
{
    m_velocity += m_force * (1 / m_mass) * dt;
    m_position += m_velocity * dt;
    m_rotation += m_rotational_velocity * dt;

    // m_force = {0.0f, 0.0f};
}
void RigidBody::draw(sf::RenderWindow &window)
{
    sf::Transform shape_transform;
    shape_transform.translate(m_position.x, m_position.y);
    shape_transform.rotate(m_rotation);
    window.draw(*m_shape_ptr, shape_transform);
}
