#pragma once
#include "rigid_body.h"

class RectangleBody : public RigidBody
{
    float m_length;
    float m_height;

    void loadShape(int scale) override;

public:
    RectangleBody(float mass, float rotation, const flatmath::Vector2 &position,
                  const flatmath::Vector2 &velocity, const flatmath::Vector2 &force,
                  float length, float height, int scale);
    RectangleBody(const RectangleBody &other);
    float getLength() const;
    float getHeight() const;
    float getShapeLength() const;
    float getShapeHeight() const;
    void getVertices(std::array<flatmath::Vector2, 4> &vertices) const;
    std::unique_ptr<RigidBody> cloneIntoPtr() const override;
};
