#include "rectangle_body.h"

#define TO_RADIANS(X) ((X) * (M_PI / 180))

RectangleBody::RectangleBody(float mass, float rotation,
                             float rotational_velocity,
                             const flatmath::Vector2 &position,
                             const flatmath::Vector2 &velocity,
                             const flatmath::Vector2 &force,
                             float length, float height,
                             int scale)
    : RigidBody(mass, rotation, rotational_velocity, position, velocity, force, scale),
      m_length(length), m_height(height)
{
    loadShape(m_scale);
}

RectangleBody::RectangleBody(const RectangleBody &other) : RigidBody(other.getMass(),
                                                                     other.getRotation(),
                                                                     other.getRotationalVelocity(),
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

float RectangleBody::getShapeLength() const
{
    return m_length * m_scale;
}

float RectangleBody::getShapeHeight() const
{
    return m_height * m_scale;
}

void RectangleBody::getVertices(std::array<flatmath::Vector2, 4> &vertices) const
{
    float cosine = cos(TO_RADIANS(m_rotation));
    float sine = sin(TO_RADIANS(m_rotation));
    float shape_length = getShapeLength();
    float shape_height = getShapeHeight();
    vertices.at(0) = {m_position.x + 0.5f * (shape_length * cosine + shape_height * sine),
                      m_position.y - 0.5f * (-shape_length * sine + shape_height * cosine)};
    vertices.at(1) = {m_position.x - 0.5f * (shape_length * cosine - shape_height * sine),
                      m_position.y - 0.5f * (shape_length * sine + shape_height * cosine)};
    vertices.at(2) = {m_position.x - 0.5f * (shape_length * cosine + shape_height * sine),
                      m_position.y + 0.5f * (-shape_length * sine + shape_height * cosine)};
    vertices.at(3) = {m_position.x + 0.5f * (shape_length * cosine - shape_height * sine),
                      m_position.y + 0.5f * (shape_length * sine + shape_height * cosine)};
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
