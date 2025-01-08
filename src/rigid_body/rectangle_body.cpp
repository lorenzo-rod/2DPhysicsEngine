#include "rectangle_body.h"

#define TO_RADIANS(X) ((X) * (M_PI / 180))

RectangleBody::RectangleBody(float length, float height)
    : RigidBody(0.f, 0.f, 0.f, 0.f, {0.f, 0.f}, {0.f, 0.f}, {0.f, 0.f}, 0.f),
      m_length(length), m_height(height)
{
    loadShape();
    loadInertiaMoment();
}

RectangleBody::RectangleBody(float inv_mass, float rotation,
                             float rotational_velocity,
                             float restitution,
                             const flatmath::Vector2 &position,
                             const flatmath::Vector2 &velocity,
                             const flatmath::Vector2 &force,
                             float torque, float length,
                             float height)
    : RigidBody(inv_mass, rotation, rotational_velocity, restitution, position, velocity, force, torque),
      m_length(length), m_height(height)
{
    loadShape();
    loadInertiaMoment();
}

RectangleBody::RectangleBody(const RectangleBody &other) : RigidBody(other.getInvMass(),
                                                                     other.getRotation(),
                                                                     other.getRotationalVelocity(),
                                                                     other.getRestitution(),
                                                                     other.getPosition(),
                                                                     other.getVelocity(),
                                                                     other.getForce(),
                                                                     other.getTorque()),
                                                           m_height(other.getHeight()),
                                                           m_length(other.getLength())
{
    loadShape();
    loadInertiaMoment();
}

float RectangleBody::getLength() const
{
    return m_length;
}

float RectangleBody::getHeight() const
{
    return m_height;
}

void RectangleBody::getVertices(std::array<flatmath::Vector2, 4> &vertices) const
{
    float cosine = cos(m_rotation);
    float sine = sin(m_rotation);
    vertices.at(0) = {m_position.x + 0.5f * (m_length * cosine + m_height * sine),
                      m_position.y - 0.5f * (-m_length * sine + m_height * cosine)};
    vertices.at(1) = {m_position.x - 0.5f * (m_length * cosine - m_height * sine),
                      m_position.y - 0.5f * (m_length * sine + m_height * cosine)};
    vertices.at(2) = {m_position.x - 0.5f * (m_length * cosine + m_height * sine),
                      m_position.y + 0.5f * (-m_length * sine + m_height * cosine)};
    vertices.at(3) = {m_position.x + 0.5f * (m_length * cosine - m_height * sine),
                      m_position.y + 0.5f * (m_length * sine + m_height * cosine)};
}

void RectangleBody::loadShape()
{
    m_shape_ptr = std::make_unique<sf::RectangleShape>(sf::Vector2f{m_length, m_height});
    m_shape_ptr->setOrigin(m_length / 2, m_height / 2);
}

void RectangleBody::loadInertiaMoment()
{
    m_inv_moment_inertia = 12.f * m_inv_mass / (m_height * m_length);
}

std::unique_ptr<RigidBody> RectangleBody::cloneIntoPtr() const
{
    return std::make_unique<RectangleBody>(*this);
}
