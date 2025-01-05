#pragma once
#include <vector>
#include "../rigid_body/rigid_body.h"
#include "../rigid_body/circle_body.h"
#include "../rigid_body/rectangle_body.h"
#include <algorithm>
#include <limits>

namespace
{
    constexpr int num_sides = 4;
    constexpr int n_obj_collision = 2;
}

class PhysicsWorld
{
    std::vector<std::unique_ptr<RigidBody>> rigid_bodies_container;

    void getNormals(std::array<flatmath::Vector2, num_sides> &normals, const std::array<flatmath::Vector2, num_sides> &vertices) const;
    bool checkCollisionsWithSAT(const std::array<std::array<flatmath::Vector2, num_sides>, n_obj_collision> &normal_arrays,
                                const std::array<std::array<flatmath::Vector2, num_sides>, n_obj_collision> &vertices,
                                flatmath::Vector2 &axis_for_resolution,
                                float &distance);
    bool checkCollisionsWithSAT(const std::array<flatmath::Vector2, num_sides> &normals,
                                const std::array<flatmath::Vector2, num_sides> &vertices,
                                const CircleBody &circle,
                                flatmath::Vector2 &axis_for_resolution,
                                float &distance);
    int getNearestVertexIndex(const flatmath::Vector2 &circle_position,
                              const std::array<flatmath::Vector2, num_sides> &vertices);
    void resolveWithImpulse(RigidBody &rigid_body_a, RigidBody &rigid_body_b,
                            const flatmath::Vector2 &axis, float distance);
    void resolveCollision(RigidBody *rigid_body_a, RigidBody *rigid_body_b);
    void resolveCollision(CircleBody &circle_a, CircleBody &circle_b);
    void resolveCollision(CircleBody &circle, RectangleBody &rectangle);
    void resolveCollision(RectangleBody &rectangle, CircleBody &circle);
    void resolveCollision(RectangleBody &rectangle_a, RectangleBody &rectangle_b);
    void resolveCollisions();
    void applyGravity(float dt);

public:
    void addRigidBody(const RigidBody &rigid_body);
    void moveRigidBody(int index, const flatmath::Vector2 &vec);
    RigidBody *getRigidBody(int index);
    size_t getNumberOfBodies() const;
    void draw(sf::RenderWindow &window);
    void step(float dt);
};
