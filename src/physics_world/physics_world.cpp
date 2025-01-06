#include "physics_world.h"

void PhysicsWorld::addRigidBody(const RigidBody &rigid_body)
{
    rigid_bodies_container.emplace_back(rigid_body.cloneIntoPtr());
}

void PhysicsWorld::moveRigidBody(int index, const flatmath::Vector2 &vec)
{
    rigid_bodies_container.at(index)->move(vec);
}

RigidBody *PhysicsWorld::getRigidBody(int index)
{
    return rigid_bodies_container.at(index).get();
}

size_t PhysicsWorld::getNumberOfBodies() const
{
    return rigid_bodies_container.size();
}

void PhysicsWorld::getNormals(std::array<flatmath::Vector2, num_sides> &normals, const std::array<flatmath::Vector2, num_sides> &vertices) const
{
    for (int i = 0; i < (num_sides - 1); i++)
    {
        normals.at(i) = vertices.at(i) - vertices.at(i + 1);
    }
    normals.at(num_sides - 1) = vertices.at(num_sides - 1) - vertices.at(0);
}

bool PhysicsWorld::checkCollisionsWithSAT(const std::array<std::array<flatmath::Vector2, num_sides>, n_obj_collision> &normal_arrays,
                                          const std::array<std::array<flatmath::Vector2, num_sides>, n_obj_collision> &vertices,
                                          flatmath::Vector2 &axis_for_resolution,
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
    distance = std::numeric_limits<float>::max();
    flatmath::Vector2 normalized_axis;

    for (const auto &normal_array : normal_arrays)
    {
        for (const auto &normal : normal_array)
        {

            normalized_axis = normal.normalize();

            for (int i = 0; i < num_sides; i++)
            {
                projections_a.at(i) = normalized_axis * vertices.at(0).at(i);
                projections_b.at(i) = normalized_axis * vertices.at(1).at(i);
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
                    axis_for_resolution = normalized_axis;
                }
            }
        }
    }

    return are_colliding;
}

bool PhysicsWorld::checkCollisionsWithSAT(const std::array<flatmath::Vector2, num_sides> &normals,
                                          const std::array<flatmath::Vector2, num_sides> &vertices,
                                          const CircleBody &circle,
                                          flatmath::Vector2 &axis_for_resolution,
                                          float &distance)
{
    bool are_colliding = true;
    std::array<float, 2> projections_circle;
    std::array<float, num_sides> projections_rectangle;
    float min_projection_circle;
    float max_projection_circle;
    float min_projection_rectangle;
    float max_projection_rectangle;
    float new_distance;
    std::array<flatmath::Vector2, num_sides + 1> axes;
    flatmath::Vector2 normalized_axis;
    flatmath::Vector2 circle_position = circle.getPosition();
    distance = std::numeric_limits<float>::max();

    for (int i = 0; i < num_sides; i++)
    {
        axes.at(i) = normals.at(i);
    }

    axes.at(num_sides) = (circle_position - vertices.at(getNearestVertexIndex(circle_position, vertices)));

    for (const auto &axis : axes)
    {

        normalized_axis = axis.normalize();

        for (int i = 0; i < num_sides; i++)
        {
            projections_rectangle.at(i) = normalized_axis * vertices.at(i);
        }

        projections_circle.at(0) = normalized_axis * (circle_position + (normalized_axis * circle.getRadius()));
        projections_circle.at(1) = normalized_axis * (circle_position - (normalized_axis * circle.getRadius()));

        min_projection_circle = *std::min_element(projections_circle.begin(), projections_circle.end());
        max_projection_circle = *std::max_element(projections_circle.begin(), projections_circle.end());
        min_projection_rectangle = *std::min_element(projections_rectangle.begin(), projections_rectangle.end());
        max_projection_rectangle = *std::max_element(projections_rectangle.begin(), projections_rectangle.end());

        if (min_projection_circle > max_projection_rectangle || min_projection_rectangle > max_projection_circle)
        {
            are_colliding = false;
            return are_colliding;
        }
        else
        {
            new_distance = std::min(max_projection_rectangle - min_projection_circle,
                                    max_projection_circle - min_projection_rectangle);
            if (new_distance < distance)
            {
                distance = new_distance;
                axis_for_resolution = normalized_axis;
            }
        }
    }

    return are_colliding;
}

int PhysicsWorld::getNearestVertexIndex(const flatmath::Vector2 &circle_position,
                                        const std::array<flatmath::Vector2, num_sides> &vertices)
{
    int min_index = 0;
    float min_distance = std::numeric_limits<float>::max();
    float distance;

    for (int i = 0; i < num_sides; i++)
    {
        distance = (circle_position - vertices.at(i)).modulus();
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = i;
        }
    }

    return min_index;
}

void PhysicsWorld::getCollisionPoint(const CircleBody &circle_a, const CircleBody &circle_b, flatmath::Vector2 &point)
{
    flatmath::Vector2 pos_a = circle_a.getPosition();
    flatmath::Vector2 ab = circle_b.getPosition() - pos_a;
    point = (pos_a + circle_a.getRadius() * ab.normalize());
}

void PhysicsWorld::getCollisionPoint(const CircleBody &circle, const RectangleBody &rectangle, flatmath::Vector2 &point)
{
    std::array<flatmath::Vector2, num_sides> vertices;
    flatmath::Vector2 cp_min_distance;
    flatmath::Vector2 cp;
    float min_squared_distance;
    float squared_distance;

    rectangle.getVertices(vertices);

    min_squared_distance = flatmath::pointToSegmentSquaredDistance(vertices.at(3),
                                                                   vertices.at(0),
                                                                   circle.getPosition(),
                                                                   cp_min_distance);

    for (int i = 0; i < (num_sides - 1); i++)
    {
        squared_distance = flatmath::pointToSegmentSquaredDistance(vertices.at(i),
                                                                   vertices.at(i + 1),
                                                                   circle.getPosition(),
                                                                   cp);
        if (squared_distance < min_squared_distance)
        {
            squared_distance = min_squared_distance;
            cp_min_distance = cp;
        }
    }
}

void PhysicsWorld::resolveWithImpulse(RigidBody &rigid_body_a, RigidBody &rigid_body_b, const flatmath::Vector2 &axis, float distance)
{
    float restitution = std::min(rigid_body_a.getRestitution(), rigid_body_b.getRestitution());
    flatmath::Vector2 vel_a = rigid_body_a.getVelocity();
    flatmath::Vector2 vel_b = rigid_body_b.getVelocity();
    flatmath::Vector2 relative_velocity = vel_a - vel_b;
    float inv_mass_a = rigid_body_a.getInvMass();
    float inv_mass_b = rigid_body_b.getInvMass();
    float impulse_magnitude = (-(1 + restitution) * (relative_velocity * axis)) /
                              (inv_mass_a + inv_mass_b);

    rigid_body_a.move(-(distance / 2) * axis);
    rigid_body_b.move((distance / 2) * axis);

    rigid_body_a.setVelocity(vel_a + (impulse_magnitude * inv_mass_a) * axis);
    rigid_body_b.setVelocity(vel_b - (impulse_magnitude * inv_mass_b) * axis);
}

void PhysicsWorld::resolveCollision(RigidBody *rigid_body_a, RigidBody *rigid_body_b)
{
    CircleBody *circle_body_a_ptr = dynamic_cast<CircleBody *>(rigid_body_a);
    CircleBody *circle_body_b_ptr = dynamic_cast<CircleBody *>(rigid_body_b);
    RectangleBody *rectangle_body_a_ptr = dynamic_cast<RectangleBody *>(rigid_body_a);
    RectangleBody *rectangle_body_b_ptr = dynamic_cast<RectangleBody *>(rigid_body_b);

    if (circle_body_a_ptr && circle_body_b_ptr)
    {
        resolveCollision(*circle_body_a_ptr, *circle_body_b_ptr);
    }
    else if (rectangle_body_a_ptr && rectangle_body_b_ptr)
    {
        resolveCollision(*rectangle_body_a_ptr, *rectangle_body_b_ptr);
    }
    else if (circle_body_a_ptr && rectangle_body_b_ptr)
    {
        resolveCollision(*circle_body_a_ptr, *rectangle_body_b_ptr);
    }
    else
    {
        resolveCollision(*rectangle_body_a_ptr, *circle_body_b_ptr);
    }

    return;
}

void PhysicsWorld::resolveCollision(CircleBody &circle_a, CircleBody &circle_b)
{
    float radius_sum = circle_a.getRadius() + circle_b.getRadius();
    float centers_distance = (circle_a.getPosition() - circle_b.getPosition()).modulus();
    flatmath::Vector2 normal = {0.f, 0.f};
    flatmath::Vector2 cp;

    if (centers_distance < radius_sum)
    {
        float distance_to_move = radius_sum - centers_distance;
        normal = (circle_b.getPosition() - circle_a.getPosition()).normalize();
        getCollisionPoint(circle_a, circle_b, cp);
        resolveWithImpulse(circle_a, circle_b, normal, distance_to_move);
    }
}

void PhysicsWorld::resolveCollision(CircleBody &circle, RectangleBody &rectangle)
{
    std::array<flatmath::Vector2, num_sides> vertices;
    std::array<flatmath::Vector2, num_sides> normals;
    flatmath::Vector2 cp;
    flatmath::Vector2 axis_for_resolution;
    float distance;

    rectangle.getVertices(vertices);
    getNormals(normals, vertices);

    if (checkCollisionsWithSAT(normals, vertices, circle, axis_for_resolution, distance))
    {

        if (axis_for_resolution * (rectangle.getPosition() - circle.getPosition()) < 0.f)
        {
            axis_for_resolution = -axis_for_resolution;
        }
        getCollisionPoint(circle, rectangle, cp);
        resolveWithImpulse(circle, rectangle, axis_for_resolution, distance);
    }
}

void PhysicsWorld::resolveCollision(RectangleBody &rectangle, CircleBody &circle)
{
    resolveCollision(circle, rectangle);
}

void PhysicsWorld::resolveCollision(RectangleBody &rectangle_a, RectangleBody &rectangle_b)
{
    std::array<flatmath::Vector2, num_sides> vertices_a;
    std::array<flatmath::Vector2, num_sides> vertices_b;
    std::array<flatmath::Vector2, num_sides> normals_sides_a;
    std::array<flatmath::Vector2, num_sides> normals_sides_b;
    flatmath::Vector2 cp;
    flatmath::Vector2 axis_for_resolution;
    float distance;

    rectangle_a.getVertices(vertices_a);
    rectangle_b.getVertices(vertices_b);

    getNormals(normals_sides_a, vertices_a);
    getNormals(normals_sides_b, vertices_b);

    if (checkCollisionsWithSAT({normals_sides_a, normals_sides_b}, {vertices_a, vertices_b}, axis_for_resolution, distance))
    {

        if (axis_for_resolution * (rectangle_b.getPosition() - rectangle_a.getPosition()) < 0.f)
        {
            axis_for_resolution = -axis_for_resolution;
        }
        resolveWithImpulse(rectangle_a, rectangle_b, axis_for_resolution, distance);
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
            resolveCollision(rigid_bodies_container.at(i).get(), rigid_bodies_container.at(j).get());
        }
    }
}

void PhysicsWorld::draw(sf::RenderWindow &window)
{
    for (const auto &rigid_body : rigid_bodies_container)
    {
        rigid_body->draw(window);
    }
}

void PhysicsWorld::applyGravity(float dt)
{
    for (auto &rigid_body : rigid_bodies_container)
    {
        rigid_body->addVelocity(flatmath::Vector2{0.f, 1000.f} * dt);
    }
}

void PhysicsWorld::step(float dt)
{
    for (auto &rigid_body : rigid_bodies_container)
    {
        rigid_body->step(dt);
    }
    resolveCollisions();
    applyGravity(dt);
}
