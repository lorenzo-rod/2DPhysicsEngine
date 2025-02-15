#pragma once
#include "rigid_body.h"

class RectangleBody : virtual public RigidBody
{
protected:
    float m_length;
    float m_height;

    void loadShape() override;
    void loadInertiaMoment() override;

public:
    RectangleBody(float length, float height);
    RectangleBody(float inv_mass, float rotation, float rotational_velocity,
                  float restitution, const flatmath::Vector2 &position,
                  const flatmath::Vector2 &velocity,
                  const flatmath::Vector2 &force, float torque,
                  float length, float height);
    RectangleBody(const RectangleBody &other);
    float getLength() const;
    float getHeight() const;
    void getVertices(std::array<flatmath::Vector2, 4> &vertices) const;
    std::unique_ptr<RigidBody> cloneIntoPtr() const override;
};
