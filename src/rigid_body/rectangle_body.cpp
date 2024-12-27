#include "rectangle_body.h"

RectangleBody::RectangleBody(float mass, float rotation,
                             const flatmath::Vector2 &position,
                             const flatmath::Vector2 &velocity,
                             const flatmath::Vector2 &force,
                             float length, float height,
                             int scale)
    : RigidBody(mass, rotation, position, velocity, force, scale), m_length(length), m_height(height)
{
    loadShape(m_scale);
}

RectangleBody::RectangleBody(const RectangleBody &other) : RigidBody(other.getMass(),
                                                                     other.getRotation(),
                                                                     other.getPosition(),
                                                                     other.getVelocity(),
                                                                     other.getForce(),
                                                                     other.getScale()),
                                                           m_height(other.getHeight()),
                                                           m_length(other.getLength())
{
    loadShape(m_scale);
}

float RectangleBody::getLength() const
{
    return m_length;
}

float RectangleBody::getHeight() const
{
    return m_height;
}

void RectangleBody::loadShape(int scale)
{
    float shape_length = m_length * scale;
    float shape_height = m_height * scale;
    m_shape_ptr = std::make_unique<sf::RectangleShape>(sf::Vector2f{shape_length, shape_height});
    m_shape_ptr->setOrigin(shape_length / 2, shape_height / 2);
}

std::unique_ptr<RigidBody> RectangleBody::cloneIntoPtr() const
{
    return std::make_unique<RectangleBody>(*this);
}
