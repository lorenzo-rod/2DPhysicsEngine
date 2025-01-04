#pragma once
#include "rectangle_body.h"
#include "static_body.h"

class StaticRectangleBody : public RectangleBody, public StaticBody
{
    float m_length;
    float m_height;

public:
    StaticRectangleBody(float rotation, float restitution,
                        const flatmath::Vector2 &position,
                        float length,
                        float height);
    StaticRectangleBody(const StaticRectangleBody &other);
    std::unique_ptr<RigidBody> cloneIntoPtr() const override;
};
