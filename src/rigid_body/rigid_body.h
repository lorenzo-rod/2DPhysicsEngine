#pragma once
#include "../math/vector2.h"
#include <SFML/Graphics.hpp>

class RigidBody
{
protected:
    float m_inv_mass;
    float m_rotation;
    float m_rotational_velocity;
    float m_restitution;
    flatmath::Vector2 m_position;
    flatmath::Vector2 m_velocity;
    flatmath::Vector2 m_force;
    std::unique_ptr<sf::Shape> m_shape_ptr;

    virtual void loadShape() = 0;

public:
    static constexpr int num_sides_box = 4;
    RigidBody(float inv_mass, float rotation, float rotational_velocity,
              float restitution, const flatmath::Vector2 &position,
              const flatmath::Vector2 &velocity,
              const flatmath::Vector2 &force);
    float getInvMass() const;
    float getRotation() const;
    float getRotationalVelocity() const;
    float getRestitution() const;
    flatmath::Vector2 getPosition() const;
    flatmath::Vector2 getVelocity() const;
    flatmath::Vector2 getForce() const;
    virtual std::array<float, num_sides_box> getAxesAlignedBoundingBox() const = 0;
    virtual void setVelocity(const flatmath::Vector2 &velocity);
    virtual void addVelocity(const flatmath::Vector2 &velocity);
    void setForce(const flatmath::Vector2 &force);
    virtual void addForce(const flatmath::Vector2 &force);
    virtual void move(const flatmath::Vector2 &vec);
    void step(float dt);
    void draw(sf::RenderWindow &window) const;
    virtual std::unique_ptr<RigidBody> cloneIntoPtr() const = 0;
};
