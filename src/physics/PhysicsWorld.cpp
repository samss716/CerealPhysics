#include "physics/PhysicsWorld.h"

// broad phase AABB
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
    resolveCollisions(); // resolve collisions after all bodies have been updated
}

// access body by its index
RigidBody &PhysicsWorld::getBody(std::size_t index)
{
    return bodies.at(index); // using at() for bounds checking
}

const RigidBody &PhysicsWorld::getBody(std::size_t index) const
{
    return bodies.at(index); // using at() for bounds checking
}

// return num of bodies
std::size_t PhysicsWorld::getBodyCount() const
{
    return bodies.size();
}

// narrow phase: sphere collision detection for cereal torus

std::vector<CollisionInfo> PhysicsWorld::detectNarrowPhaseCollisions() const
{
    // getting all potentially colliding pairs of bodies in the physics world from broad phase
    auto potentialCollisions = detectCollisions();

    // storing confirmed collision information from collision.h
    std::vector<CollisionInfo> confirmedCollisions;

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
            // calculate collision normal
            Vector3 normal;

            if (distance > 1e-8) // avoid division by zero
            {
                normal = delta / distance; // normalize delta to get collision normal
            }
            else
            {
                // If the distance is zero, we can choose an arbitrary normal (e.g., along the x-axis)
                normal = Vector3(1.0, 0.0, 0.0);
            }

            // calculate penetration depth
            double penetration = combinedRadius - distance;

            // store collision information
            confirmedCollisions.push_back({pair.first, pair.second, normal, penetration});
        }
    }

    return confirmedCollisions;
}
// Collision response: Impulse solver
void PhysicsWorld::resolveCollisions(double restitution)
{
    auto collisions = detectNarrowPhaseCollisions(); // getting all confirmed collisions from narrow phase

    for (const CollisionInfo &collision : collisions)
    {
        RigidBody &bodyA = bodies[collision.bodyAIndex];
        RigidBody &bodyB = bodies[collision.bodyBIndex];

        // Calculate relative velocity
        Vector3 relativeVelocity = bodyB.getVelocity() - bodyA.getVelocity();

        // Calculate relative velocity along the normal with dot product
        double velocityAlongNormal = glm::dot(relativeVelocity, collision.normal);

        // Only resolve if bodies are moving towards each other
        if (velocityAlongNormal >= 0.0)
        {
            continue;
        } // if moving away, skip

        // Calculate impulse scalar/inverse massess
        double inverseMassA = 1.0 / bodyA.getMass();
        double inverseMassB = 1.0 / bodyB.getMass();

        // calc impulse magnitude
        double impulseMagnitude = -(1.0 + restitution) * velocityAlongNormal / (inverseMassA + inverseMassB);

        // convert scalar impulse into a vector
        //  Apply impulse to both bodies
        Vector3 impulse = impulseMagnitude * collision.normal;

        // apply equal and opposite impulses to the two bodies
        bodyA.applyImpulse(-impulse);
        bodyB.applyImpulse(impulse);
    }
}