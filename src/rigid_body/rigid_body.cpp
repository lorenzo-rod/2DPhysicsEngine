#include "rigid_body.h"

inline float to_degrees(float x) { return x * 180.f / M_PI; };

RigidBody::RigidBody(float inv_mass, float rotation,
                     float rotational_velocity,
                     float restitution,
                     const flatmath::Vector2 &position,
                     const flatmath::Vector2 &velocity,
                     const flatmath::Vector2 &force,
                     float torque)
    : m_inv_mass(inv_mass),
      m_rotation(rotation),
      m_rotational_velocity(rotational_velocity),
      m_restitution(restitution),
      m_position(position),
      m_velocity(velocity),
      m_force(force),
      m_torque(torque)
{
}

RigidBody::~RigidBody()
{
}

float RigidBody::getInvMass() const
{
    return m_inv_mass;
}
float RigidBody::getInvInertiaMoment() const
{
    return m_inv_moment_inertia;
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
float RigidBody::getTorque() const
{
    return m_torque;
}
void RigidBody::addForce(const flatmath::Vector2 &force)
{
    m_force += force;
}
void RigidBody::setVelocity(const flatmath::Vector2 &velocity)
{
    m_velocity = velocity;
}
void RigidBody::addVelocity(const flatmath::Vector2 &velocity)
{
    m_velocity += velocity;
}
void RigidBody::setRotationalVelocity(float rot_vel)
{
    m_rotational_velocity = rot_vel;
}
void RigidBody::addRotationalVelocity(float rot_vel)
{
    m_rotational_velocity += rot_vel;
}
void RigidBody::setForce(const flatmath::Vector2 &force)
{
    m_force = force;
}
void RigidBody::move(const flatmath::Vector2 &vec)
{
    m_position += vec;
}
void RigidBody::step(float dt)
{
    m_velocity += m_force * m_inv_mass * dt;
    m_position += m_velocity * dt;
    m_rotational_velocity += m_torque * m_inv_moment_inertia * dt;
    m_rotation += m_rotational_velocity * dt;
}
void RigidBody::draw(sf::RenderWindow &window) const
{
    sf::Transform shape_transform;
    shape_transform.translate(m_position.x, m_position.y);
    shape_transform.rotate(to_degrees(m_rotation));
    window.draw(*m_shape_ptr, shape_transform);
}
