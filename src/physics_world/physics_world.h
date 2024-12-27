#pragma once
#include <vector>
#include "../rigid_body/rigid_body.h"

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

    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;
};
