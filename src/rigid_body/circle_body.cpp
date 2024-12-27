#include "circle_body.h"

CircleBody::CircleBody(float mass, float rotation,
                       const flatmath::Vector2 &position,
                       const flatmath::Vector2 &velocity,
                       const flatmath::Vector2 &force,
                       float radius, int scale)
    : RigidBody(mass, rotation, position, velocity, force), m_radius(radius)
{
    loadShape(scale);
}

void CircleBody::loadShape(int scale)
{
    float shape_radius = m_radius * scale;
    m_shape_ptr = std::make_unique<sf::CircleShape>(shape_radius);
    m_shape_ptr->setOrigin(shape_radius, shape_radius);
}
