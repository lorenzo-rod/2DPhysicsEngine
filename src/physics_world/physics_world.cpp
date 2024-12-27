#include "physics_world.h"

void PhysicsWorld::addRigidBody(const RigidBody &rigid_body)
{
    rigid_bodies_container.emplace_back(rigid_body.cloneIntoPtr());
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
