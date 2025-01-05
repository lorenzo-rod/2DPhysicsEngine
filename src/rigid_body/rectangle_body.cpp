#include "rectangle_body.h"

#define TO_RADIANS(X) ((X) * (M_PI / 180))

RectangleBody::RectangleBody(float length, float height)
    : RigidBody(0.f, 0.f, 0.f, 0.f, {0.f, 0.f}, {0.f, 0.f}, {0.f, 0.f}),
      m_length(length), m_height(height)
{
    loadShape();
}

RectangleBody::RectangleBody(float inv_mass, float rotation,
                             float rotational_velocity,
                             float restitution,
                             const flatmath::Vector2 &position,
                             const flatmath::Vector2 &velocity,
                             const flatmath::Vector2 &force,
                             float length, float height)
    : RigidBody(inv_mass, rotation, rotational_velocity, restitution, position, velocity, force),
      m_length(length), m_height(height)
{
    loadShape();
}

RectangleBody::RectangleBody(const RectangleBody &other) : RigidBody(other.getInvMass(),
                                                                     other.getRotation(),
                                                                     other.getRotationalVelocity(),
                                                                     other.getRestitution(),
                                                                     other.getPosition(),
                                                                     other.getVelocity(),
                                                                     other.getForce()),
                                                           m_height(other.getHeight()),
                                                           m_length(other.getLength())
{
    loadShape();
}

float RectangleBody::getLength() const
{
    return m_length;
}

float RectangleBody::getHeight() const
{
    return m_height;
}

void RectangleBody::getVertices(std::array<flatmath::Vector2, RigidBody::num_sides_box> &vertices) const
{
    float cosine = cos(TO_RADIANS(m_rotation));
    float sine = sin(TO_RADIANS(m_rotation));
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

std::array<float, RigidBody::num_sides_box> RectangleBody::getAxesAlignedBoundingBox() const
{
    std::array<flatmath::Vector2, num_sides_box> vertices;
    std::array<float, num_sides_box> vertices_x;
    std::array<float, num_sides_box> vertices_y;
    float min_x;
    float max_x;
    float min_y;
    float max_y;

    getVertices(vertices);

    min_x = std::min({vertices.at(0).x, vertices.at(1).x, vertices.at(2).x, vertices.at(3).x});
    max_x = std::max({vertices.at(0).x, vertices.at(1).x, vertices.at(2).x, vertices.at(3).x});
    min_y = std::min({vertices.at(0).y, vertices.at(1).y, vertices.at(2).y, vertices.at(3).y});
    max_y = std::max({vertices.at(0).y, vertices.at(1).y, vertices.at(2).y, vertices.at(3).y});

    return std::array<float, num_sides_box>{min_x, max_x, min_y, max_y};
}

std::unique_ptr<RigidBody> RectangleBody::cloneIntoPtr() const
{
    return std::make_unique<RectangleBody>(*this);
}
