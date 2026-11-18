#include "physics/PhysicsWorld.h"

std::vector<std::pair<std::size_t, std::size_t>> PhysicsWorld::detectCollisions() const
{
    std::vector<std::pair<std::size_t, std::size_t>> collidingPairs;
    // compare every unique pair of rigid bodies in the world to see if their AABBs overlap
    for (std::size_t i = 0; i < bodies.size(); ++i)
    {
        for (std::size_t j = i + 1; j < bodies.size(); ++j) // avoid checking same pair twice
        {
            AABB boxA = bodies[i].getAABB();
            AABB boxB = bodies[j].getAABB();

            if (boxA.overlaps(boxB))
            {
                collidingPairs.emplace_back(i, j); // emplace:
            }
        }
    }

    return collidingPairs;
}

// Constructor
PhysicsWorld::PhysicsWorld(const Vector3 &gravity)
    : gravity(gravity)
{
}

// adding rigid body to sim
void PhysicsWorld::addBody(double mass, const Vector3 &initialPosition)
{
    bodies.emplace_back(mass, initialPosition); // construct RigidBody inside vector
}

// Advance sim by one timestep
void PhysicsWorld::step(double deltaTime)
{
    for (RigidBody &body : bodies) // sim loop; access by reference
    {
        // calc gravitational force: F=m*g
        Vector3 gravityForce = gravity * body.getMass();
        body.applyForce(gravityForce);
        body.integrate(deltaTime);
    }
}

// access body by its index
RigidBody &PhysicsWorld::getBody(std::size_t index)
{
    return bodies[index];
}

const RigidBody &PhysicsWorld::getBody(std::size_t index) const
{
    return bodies[index];
}

// return num of bodies
std::size_t PhysicsWorld::getBodyCount() const
{
    return bodies.size();
}