#include "physics_world.h"

void PhysicsWorld::addRigidBody(const RigidBody &rigid_body)
{
    rigid_bodies_container.emplace_back(rigid_body.cloneIntoPtr());
}

void PhysicsWorld::moveRigidBody(int index, const flatmath::Vector2 &vec)
{
    rigid_bodies_container.at(index)->move(vec);
}

void PhysicsWorld::getNormals(std::array<flatmath::Vector2, num_sides> &normals, const std::array<flatmath::Vector2, num_sides> &vertices) const
{
    for (int i = 0; i < 3; i++)
    {
        normals.at(i) = vertices.at(i) - vertices.at(i + 1);
    }
    normals.at(3) = vertices.at(3) - vertices.at(0);
}

bool PhysicsWorld::checkCollisionsWithSAT(const std::array<std::array<flatmath::Vector2, num_sides>, n_obj_collision> &normal_arrays,
                                          const std::array<std::array<flatmath::Vector2, num_sides>, n_obj_collision> &vertices,
                                          flatmath::Vector2 &normal_for_resolution,
                                          float &distance)
{
    bool are_colliding = true;
    std::array<float, num_sides> projections_a;
    std::array<float, num_sides> projections_b;
    float min_projection_a;
    float max_projection_a;
    float min_projection_b;
    float max_projection_b;
    float new_distance;

    for (const auto &normal_array : normal_arrays)
    {
        for (const auto &normal : normal_array)
        {
            for (int i = 0; i < num_sides; i++)
            {
                projections_a.at(i) = normal * vertices.at(0).at(i);
                projections_b.at(i) = normal * vertices.at(1).at(i);
            }

            min_projection_a = *std::min_element(projections_a.begin(), projections_a.end());
            max_projection_a = *std::max_element(projections_a.begin(), projections_a.end());
            min_projection_b = *std::min_element(projections_b.begin(), projections_b.end());
            max_projection_b = *std::max_element(projections_b.begin(), projections_b.end());

            if (min_projection_a > max_projection_b || min_projection_b > max_projection_a)
            {
                are_colliding = false;
                return are_colliding;
            }
            else
            {
                new_distance = std::min(max_projection_b - min_projection_a, max_projection_a - min_projection_b);
                if (new_distance < distance)
                {
                    distance = new_distance;
                    normal_for_resolution = normal;
                }
            }
        }
    }

    return are_colliding;
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
    RectangleBody *rectangle_body_a_ptr = dynamic_cast<RectangleBody *>(&rigid_body_a);
    RectangleBody *rectangle_body_b_ptr = dynamic_cast<RectangleBody *>(&rigid_body_b);

    if (circle_body_a_ptr && circle_body_b_ptr)
    {
        resolveCollision(*circle_body_a_ptr, *circle_body_b_ptr);
    }
    else if (rectangle_body_a_ptr && rectangle_body_b_ptr)
    {
        resolveCollision(*rectangle_body_a_ptr, *rectangle_body_b_ptr);
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
    std::array<flatmath::Vector2, num_sides> vertices_a;
    std::array<flatmath::Vector2, num_sides> vertices_b;
    std::array<flatmath::Vector2, num_sides> normals_sides_a;
    std::array<flatmath::Vector2, num_sides> normals_sides_b;
    flatmath::Vector2 normal_for_resolution;
    float distance = std::numeric_limits<float>::max();

    rectangle_a.getVertices(vertices_a);
    rectangle_b.getVertices(vertices_b);

    getNormals(normals_sides_a, vertices_a);
    getNormals(normals_sides_b, vertices_b);

    if (checkCollisionsWithSAT({normals_sides_a, normals_sides_b}, {vertices_a, vertices_b}, normal_for_resolution, distance))
    {
        distance /= normal_for_resolution.modulus();
        normal_for_resolution = normal_for_resolution.normalize();

        if (normal_for_resolution * (rectangle_b.getPosition() - rectangle_a.getPosition()) < 0.f)
        {
            normal_for_resolution = -normal_for_resolution;
        }

        rectangle_a.move(-(distance / 2) * normal_for_resolution);
        rectangle_b.move((distance / 2) * normal_for_resolution);
    }
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
