#pragma once
#include "../math/point2.h"
#include "../math/vector2.h"
#include <SFML/Graphics.hpp>

class RigidBody
{
protected:
    float m_mass;
    float m_rotation;
    flatmath::Vector2 m_position;
    flatmath::Vector2 m_velocity;
    flatmath::Vector2 m_force;
    std::unique_ptr<sf::Shape> m_shape_ptr;
    int m_scale;

public:
    RigidBody(float mass, float rotation, const flatmath::Vector2 &position,
              const flatmath::Vector2 &velocity, const flatmath::Vector2 &force,
              int scale);
    float getMass() const;
    float getRotation() const;
    flatmath::Vector2 getPosition() const;
    flatmath::Vector2 getVelocity() const;
    flatmath::Vector2 getForce() const;
    int getScale() const;
    void step(float dt);
    void draw(sf::RenderWindow &window);
    virtual void loadShape(int scale) = 0;
    virtual std::unique_ptr<RigidBody> cloneIntoPtr() const = 0;
};
