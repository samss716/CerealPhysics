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
                collidingPairs.emplace_back(i, j); // emplace: construct pair in place
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

std::vector<std::pair<std::size_t, std::size_t>> PhysicsWorld::detectNarrowPhaseCollisions() const
{
    // getting all potentially colliding pairs of bodies in the physics world from broad phase
    auto potentialCollisions = detectCollisions();

    // storing confirmed collision paris
    std::vector<std::pair<std::size_t, std::size_t>> confirmedCollisions;

    for (const auto &pair : potentialCollisions)
    {
        const RigidBody &bodyA = bodies[pair.first];
        const RigidBody &bodyB = bodies[pair.second];

        // Calculate the distance between the two bodies
        Vector3 delta = bodyB.getPosition() - bodyA.getPosition(); // calcs vector from bodyA to bodyBs

        double distance = glm::length(delta);

        // Calculate the sum of their collision radii
        double combinedRadius = bodyA.getCollisionRadius() + bodyB.getCollisionRadius();

        // collision detect if sphere overlap
        if (distance < combinedRadius)
        {
            confirmedCollisions.emplace_back(pair.first, pair.second);
        }
    }
    return confirmedCollisions;
}