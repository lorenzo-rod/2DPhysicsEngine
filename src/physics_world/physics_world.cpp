#include "physics_world.h"

void PhysicsWorld::addRigidBody(const RigidBody &rigid_body)
{
    rigid_bodies_container.emplace_back(rigid_body.cloneIntoPtr());
}

void PhysicsWorld::moveRigidBody(int index, const flatmath::Vector2 &vec)
{
    rigid_bodies_container.at(index)->move(vec);
}

iterator PhysicsWorld::begin()
{
    return rigid_bodies_container.begin();
}
const_iterator PhysicsWorld::begin() const
{
    return rigid_bodies_container.begin();
}
iterator PhysicsWorld::end()
{
    return rigid_bodies_container.end();
}
const_iterator PhysicsWorld::end() const
{
    return rigid_bodies_container.end();
}

void PhysicsWorld::resolveCollision(RigidBody &rigid_body_a, RigidBody &rigid_body_b)
{
    CircleBody *circle_body_a_ptr = dynamic_cast<CircleBody *>(&rigid_body_a);
    CircleBody *circle_body_b_ptr = dynamic_cast<CircleBody *>(&rigid_body_b);

    if (circle_body_a_ptr && circle_body_b_ptr)
    {
        resolveCollision(*circle_body_a_ptr, *circle_body_b_ptr);
    }

    // TO DO: Implement other collisions

    return;
}

void PhysicsWorld::resolveCollision(CircleBody &circle_a, CircleBody &circle_b)
{
    float radius_sum = circle_a.getRadius() * circle_a.getScale() + circle_b.getRadius() * circle_b.getScale();
    float distance = (circle_a.getPosition() - circle_b.getPosition()).modulus();
    flatmath::Vector2 normal = {0.f, 0.f};

    if (distance < radius_sum)
    {
        float distance_to_move = radius_sum - distance;
        normal = (circle_b.getPosition() - circle_a.getPosition()).normalize();
        circle_a.move(-(distance_to_move / 2) * normal);
        circle_b.move((distance_to_move / 2) * normal);
    }
}

void PhysicsWorld::resolveCollision(CircleBody &circle, RectangleBody &rectangle)
{
    // TO DO
}

void PhysicsWorld::resolveCollision(RectangleBody &rectangle, CircleBody &circle)
{
    // TO DO
}

void PhysicsWorld::resolveCollision(RectangleBody &rectangle_a, RectangleBody &rectangle_b)
{
    // TO DO
}

void PhysicsWorld::resolveCollisions()
{
    for (size_t i = 0; i < rigid_bodies_container.size(); i++)
    {
        for (size_t j = 0; j < rigid_bodies_container.size(); j++)
        {
            if (i == j)
                break;
            resolveCollision(*(rigid_bodies_container.at(i)), *(rigid_bodies_container.at(j)));
        }
    }
}
