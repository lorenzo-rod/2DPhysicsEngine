#pragma once
#include <vector>
#include "../rigid_body/rigid_body.h"
#include "../rigid_body/circle_body.h"
#include "../rigid_body/rectangle_body.h"

namespace
{
    using iterator = std::vector<std::unique_ptr<RigidBody>>::iterator;
    using const_iterator = std::vector<std::unique_ptr<RigidBody>>::const_iterator;
}

class PhysicsWorld
{
    std::vector<std::unique_ptr<RigidBody>> rigid_bodies_container;

public:
    void addRigidBody(const RigidBody &rigid_body);
    void moveRigidBody(int index, const flatmath::Vector2 &vec);
    void getNormals(std::array<flatmath::Vector2, 4> &normals, const std::array<flatmath::Vector2, 4> &vertices) const;
    void resolveCollision(RigidBody &rigid_body_a, RigidBody &rigid_body_b);
    void resolveCollision(CircleBody &circle_a, CircleBody &circle_b);
    void resolveCollision(CircleBody &circle, RectangleBody &rectangle);
    void resolveCollision(RectangleBody &rectangle, CircleBody &circle);
    void resolveCollision(RectangleBody &rectangle_a, RectangleBody &rectangle_b);
    void resolveCollisions();
    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;
};
