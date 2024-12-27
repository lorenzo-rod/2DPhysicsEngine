#pragma once
#include "rigid_body.h"

class RectangleBody : public RigidBody
{
    float m_length;
    float m_height;

public:
    RectangleBody(float mass, float rotation, const flatmath::Vector2 &position,
                  const flatmath::Vector2 &velocity, const flatmath::Vector2 &force,
                  float length, float height, int scale);
    void loadShape(int scale) override;
};